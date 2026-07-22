#!/usr/bin/env python3
"""
PX4 MAVLink 飞行控制测试脚本
==============================

功能: Zero3W 通过串口直接对 PX4 发送飞行控制指令。
      支持交互式菜单和自动测试序列。
      --hud 模式通过后台遥测线程持续更新 HUD。

硬件:  Zero3W UART3 (/dev/ttyS3) → PX4 TELEM1
协议:  MAVLink v2, 115200 8N1

用法:
  # 交互式菜单 (默认)
  python3 test_mavlink_control.py

  # 带 HUD 显示
  sudo python3 test_mavlink_control.py --hud

  # 自动测试序列
  sudo python3 test_mavlink_control.py --mode guided
  sudo python3 test_mavlink_control.py --mode offboard
  sudo python3 test_mavlink_control.py --mode full

安全:
  - 遥控器保持开启作为紧急备份 (打杆自动退出 OFFBOARD)
  - Ctrl+C 任何时候都会尝试 Land + Disarm
  - 地面不会执行 ARM (需要确认)

依赖: pymavlink, pyserial, opencv-python, numpy
"""

import argparse
import math
import os
import struct
import sys
import threading
import time
from collections import deque

try:
    from pymavlink import mavutil
except ImportError:
    print("[FATAL] pymavlink 未安装。")
    print("  安装: pip3 install --break-system-packages pymavlink")
    sys.exit(1)


# ============================================================
# 常量
# ============================================================

# PX4 模式
PX4_MODE = {
    "MANUAL":       0x00000000,
    "ALTCTL":       0x00010000,
    "POSCTL":       0x00030000,
    "AUTO":         0x00040000,
    "AUTO_LOITER":  0x00050000,
    "AUTO_RTL":     0x00080000,
    "ACRO":         0x00090000,
    "OFFBOARD":     0x00060000,
    "STABILIZED":   0x000E0000,
    "RATTITUDE":    0x00110000,
    "AUTO_TAKEOFF": 0x00140000,
    "AUTO_LAND":    0x00150000,
    "AUTO_FOLLOW":  0x00160000,
    "GUIDED":       0x00040000,
}

# MAVLink 常量
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# 默认连接参数
DEFAULT_PORT = "/dev/ttyS3"
DEFAULT_BAUD = 115200


# ============================================================
# 辅助函数
# ============================================================

def print_banner():
    print("")
    print("╔" + "═" * 58 + "╗")
    print("║" + "  PX4 MAVLink 飞行控制测试工具".center(54) + "║")
    print("╚" + "═" * 58 + "╝")


def print_sep(title=""):
    if title:
        print("\n" + "─" * 50)
        print(f"  {title}")
        print("─" * 50)
    else:
        print("─" * 50)


def status_emoji(ok):
    return "✅" if ok else "❌"


def px4_main_mode_name(custom_mode):
    """从 PX4 custom_mode 提取主模式名。"""
    main_v1 = (custom_mode >> 16) & 0xFF
    sub_v1  = (custom_mode >> 24) & 0xFF
    main_v0 = custom_mode & 0xFF
    sub_v0  = (custom_mode >> 8) & 0xFF

    names = {
        0: "MANUAL", 1: "ALTCTL", 2: "POSCTL",
        3: "AUTO",   4: "AUTO",   5: "LOITER",
        6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
    }
    auto_subs = {4: "LOITER", 5: "LOITER", 8: "RTL", 20: "TAKEOFF", 21: "LAND", 22: "FOLLOW"}

    if 0 <= main_v1 <= 20:
        name = names.get(main_v1, f"MAIN{main_v1}")
        if main_v1 in (3, 4):
            sub_name = auto_subs.get(sub_v1, f"SUB{sub_v1}")
            return f"AUTO.{sub_name}"
        return name

    name = names.get(main_v0, f"MAIN{main_v0}")
    if main_v0 == 3:
        sub_name = auto_subs.get(sub_v0, f"SUB{sub_v0}")
        return f"AUTO.{sub_name}"
    return name


def mav_state_name(state):
    return {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING",
            3: "STANDBY", 4: "ACTIVE", 5: "CRITICAL",
            6: "EMERGENCY", 7: "POWEROFF", 8: "FLIGHT_TERMINATION"}.get(state, f"?{state}")


def _ned_motion_name(vx, vy, vz):
    """NED 速度 → 人类可读的运动描述"""
    parts = []
    if vx > 0.1:   parts.append("FWD")
    elif vx < -0.1: parts.append("BACK")
    if vy > 0.1:   parts.append("RIGHT")
    elif vy < -0.1: parts.append("LEFT")
    if vz > 0.1:   parts.append("DOWN")
    elif vz < -0.1: parts.append("UP")
    if not parts:
        parts.append("HOVER")
    return "+".join(parts)


# ============================================================
# PX4Controller 类
# ============================================================

class PX4Controller:
    """PX4 飞行控制器 — 后台遥测线程 + 控制指令。

    架构:
      后台遥测线程 — 持续读串口 (parse_buffer) → 更新 _shared_telemetry
      主线程 — 发指令 + 交互菜单，需要遥测时直接读共享缓存
      HUD 线程 — 读共享缓存 → 渲染 → 写 /dev/fb0

    串口只有一个读取者 (后台线程)，避免了数据竞争。
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.mav = None
        self._running = True

        # ACK 队列 (后台遥测线程写入，主线程消费)
        self._ack_queue = []
        self._ack_lock = threading.Lock()

        # ── 共享遥测 (后台遥测线程写入，HUD/主线程只读) ──
        self._shared_telemetry = {
            "connected": False,
            "armed": None, "mode": None, "mode_name": "?",
            "state": None, "state_name": "?",
            "alt_rel": None, "vx": None, "vy": None, "vz": None,
            "battery_v": None,
            "roll": None, "pitch": None, "yaw": None,
            "roll_deg": None, "pitch_deg": None, "yaw_deg": None,
            "ekf_ok": None,
        }
        self._telemetry_lock = threading.Lock()

        # ── 后台遥测线程 ──
        self._telemetry_thread = None
        self._telemetry_running = False

        # ── 日志缓存 ──
        self._log_lines = deque(maxlen=20)

    # ── 连接 ──────────────────────────────────────────

    def connect(self):
        print(f"[INFO] 连接 {self.port} @ {self.baud}...")
        try:
            self.mav = mavutil.mavlink_connection(
                device=self.port,
                baud=self.baud,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )
        except Exception as e:
            print(f"[FAIL] 无法打开串口: {e}")
            return False
        print(f"  {status_emoji(True)} 串口打开成功")
        return True

    def close(self):
        self._running = False
        self.stop_telemetry_thread()
        if self.mav:
            self.mav.close()

    # ── 后台遥测线程 ──────────────────────────────────

    def start_telemetry_thread(self):
        """启动后台遥测线程：持续读串口 → 解析 → 更新 _shared_telemetry。

        该线程是串口的唯一读取者。所有 MAVLink 消息解析后
        写入共享缓存，供 HUD 渲染线程和主线程使用。
        """
        if self._telemetry_running:
            return
        if self.mav is None:
            print("[WARN] 串口未连接，无法启动遥测线程")
            return

        self._telemetry_running = True
        self.mav.port.timeout = 0  # 非阻塞

        def _loop():
            parser = self.mav.mav
            port = self.mav.port
            while self._telemetry_running and self._running:
                try:
                    w = port.in_waiting
                except Exception:
                    w = 0
                if w > 0:
                    try:
                        data = port.read(min(w, 2048))
                        msgs = parser.parse_buffer(data)
                        if msgs:
                            for msg in msgs:
                                self._on_telemetry_msg(msg)
                    except Exception:
                        pass
                else:
                    time.sleep(0.002)

        self._telemetry_thread = threading.Thread(target=_loop, daemon=True)
        self._telemetry_thread.start()
        print("[INFO] 后台遥测线程已启动")

    def stop_telemetry_thread(self):
        self._telemetry_running = False
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=2)

    def _on_telemetry_msg(self, msg):
        """解析单条 MAVLink 消息 → 更新共享缓存 + ACK 队列。"""
        t = msg.get_type()

        if t == "HEARTBEAT":
            with self._telemetry_lock:
                self._shared_telemetry["connected"] = True
                self._shared_telemetry["armed"] = \
                    (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self._shared_telemetry["mode"] = msg.custom_mode
                self._shared_telemetry["mode_name"] = px4_main_mode_name(msg.custom_mode)
                self._shared_telemetry["state"] = msg.system_status
                self._shared_telemetry["state_name"] = mav_state_name(msg.system_status)

        elif t == "ATTITUDE":
            with self._telemetry_lock:
                self._shared_telemetry["roll"] = msg.roll
                self._shared_telemetry["pitch"] = msg.pitch
                self._shared_telemetry["yaw"] = msg.yaw
                self._shared_telemetry["roll_deg"] = math.degrees(msg.roll)
                self._shared_telemetry["pitch_deg"] = math.degrees(msg.pitch)
                self._shared_telemetry["yaw_deg"] = math.degrees(msg.yaw)

        elif t == "LOCAL_POSITION_NED":
            with self._telemetry_lock:
                self._shared_telemetry["vx"] = msg.vx
                self._shared_telemetry["vy"] = msg.vy
                self._shared_telemetry["vz"] = msg.vz

        elif t == "GLOBAL_POSITION_INT":
            with self._telemetry_lock:
                self._shared_telemetry["alt_rel"] = msg.relative_alt / 1000.0

        elif t == "BATTERY_STATUS":
            if msg.voltages:
                v = msg.voltages[0] / 1000.0
                if v < 100:
                    with self._telemetry_lock:
                        self._shared_telemetry["battery_v"] = v

        elif t == "ESTIMATOR_STATUS":
            flags = getattr(msg, "health_flags", None) or getattr(msg, "flags", 0)
            ekf_ok = (flags & 0x01) != 0 if flags else False
            with self._telemetry_lock:
                self._shared_telemetry["ekf_ok"] = ekf_ok

        elif t == "COMMAND_ACK":
            with self._ack_lock:
                self._ack_queue.append({
                    "command": msg.command,
                    "result": msg.result,
                })

        elif t == "STATUSTEXT":
            sev = {0: "E", 1: "A", 2: "C", 3: "ERR", 4: "WARN", 5: "N", 6: "INFO"}
            text = msg.text[:50] if isinstance(msg.text, str) else str(msg.text)[:50]
            hud = getattr(self, "_hud", None)
            if hud:
                hud._log(f"[{sev.get(msg.severity, '?')}] {text}")

    def _log(self, text):
        """写日志到 HUD"""
        hud = getattr(self, "_hud", None)
        if hud:
            hud._log(text)

    # ── 心跳等待 ──────────────────────────────────────

    def wait_for_heartbeat(self, timeout=15):
        """等待 PX4 心跳 (从共享缓存检查)。"""
        print(f"[INFO] 等待 PX4 心跳 (最多 {timeout}s)...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._telemetry_lock:
                if self._shared_telemetry.get("connected"):
                    tele = dict(self._shared_telemetry)
                    print(f"  {status_emoji(True)} 已连接 PX4 "
                          f"(mode={tele.get('mode_name', '?')}, "
                          f"armed={'YES' if tele.get('armed') else 'NO'})")
                    return True
            time.sleep(0.2)
        print(f"  {status_emoji(False)} 超时: 未收到心跳")
        return False

    # ── 遥测读取 ──────────────────────────────────────

    def read_telemetry(self, timeout=0.0):
        """返回共享遥测缓存的原子快照。

        后台遥测线程持续更新，此方法无需阻塞读串口。
        timeout 参数保留以兼容旧调用。
        """
        with self._telemetry_lock:
            return dict(self._shared_telemetry)

    def print_status(self):
        """打印当前飞控状态 (从共享缓存读取)"""
        tele = self.read_telemetry()
        print_sep("飞控状态")
        print(f"  Armed:         {'是' if tele.get('armed') else '否'}")
        mode = tele.get("mode")
        mode_name = tele.get("mode_name", "?")
        if mode is not None:
            print(f"  模式:          {mode_name} (0x{mode:08X})")
        else:
            print(f"  模式:          ?")
        state = tele.get("state")
        state_name = tele.get("state_name", "?")
        if state is not None:
            print(f"  系统状态:      {state_name} ({state})")
        alt = tele.get("alt_rel")
        if alt is not None:
            print(f"  相对高度:      {alt:.1f} m")
        vx = tele.get("vx")
        if vx is not None:
            print(f"  速度 NED:      vx={vx:.1f}, vy={tele.get('vy', 0):.1f}, "
                  f"vz={tele.get('vz', 0):.1f} m/s")
        bat = tele.get("battery_v")
        if bat is not None:
            print(f"  电池:          {bat:.1f} V")
        roll = tele.get("roll")
        if roll is not None:
            print(f"  姿态 (r/p/y):  {math.degrees(roll):.0f}° "
                  f"{math.degrees(tele.get('pitch', 0)):.0f}° "
                  f"{math.degrees(tele.get('yaw', 0)):.0f}°")
        ekf = tele.get("ekf_ok")
        if ekf is not None:
            print(f"  EKF:           {'OK' if ekf else 'OFF'}")
        print_sep()
        return tele

    # ── COMMAND_ACK 队列操作 ──────────────────────────

    def _drain_ack(self, command_id):
        """从 ACK 队列中取指定命令的结果。返回 True/False/None。"""
        with self._ack_lock:
            for i, ack in enumerate(self._ack_queue):
                if ack["command"] == command_id:
                    ok = (ack["result"] == 0)  # MAV_RESULT_ACCEPTED
                    self._ack_queue.pop(i)
                    return ok
        return None

    def wait_ack(self, command_id, timeout=5.0):
        """等待指定命令的 ACK (从共享队列读取)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._drain_ack(command_id)
            if result is not None:
                return result
            time.sleep(0.1)
        return None  # 超时

    # ── HUD 命令日志 ──────────────────────────────────

    def _hud_log_cmd(self, cmd_name, result):
        """更新 HUD 命令状态"""
        cs = getattr(self, "_hud_cmd_state", None)
        if cs is None:
            return
        cs["last_cmd"] = cmd_name
        cs["last_result"] = result
        cs["history"].append({
            "time": time.strftime("%H:%M:%S"),
            "cmd": cmd_name,
            "result": result,
        })
        if len(cs["history"]) > 10:
            cs["history"] = cs["history"][-8:]
        self._log(f"TX: {cmd_name} -> {result}")

    # ── 模式切换 ──────────────────────────────────────

    def set_mode(self, mode_name):
        """切换到指定 PX4 模式。通过共享遥测 + ACK 队列检测结果。"""
        mode_upper = mode_name.upper()

        mode_map = {
            "GUIDED": 0x00040004, "OFFBOARD": 0x00060006,
            "POSCTL": 0x00030002, "ALTCTL": 0x00010001,
            "STABILIZED": 0x000E0007, "MANUAL": 0x00000000,
            "RTL": 0x00080008, "LAND": 0x00150015, "LOITER": 0x00050005,
        }
        custom_mode = mode_map.get(mode_upper)
        if custom_mode is None:
            print(f"[FAIL] 未知模式: {mode_name}")
            return False

        print(f"[INFO] 切换模式 → {mode_upper}...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode, 0, 0, 0, 0, 0,
        )

        deadline = time.time() + 5.0
        while time.time() < deadline:
            # 1) 检查 ACK 队列
            ack = self._drain_ack(176)  # MAV_CMD_DO_SET_MODE
            if ack is True:
                print(f"  {status_emoji(True)} SET_MODE ACCEPTED")
                time.sleep(0.3)
                return True
            elif ack is False:
                print(f"  {status_emoji(False)} SET_MODE REJECTED")
                self._hud_log_cmd(f"SET_MODE {mode_upper}", "REJECTED")
                return False

            # 2) 检查共享遥测中的模式是否已变化
            with self._telemetry_lock:
                current = self._shared_telemetry.get("mode_name", "?")
            if mode_upper == "GUIDED" and "AUTO" in str(current):
                print(f"  {status_emoji(True)} 模式已切换为 {current}")
                self._hud_log_cmd(f"SET_MODE {mode_upper}", "ACCEPTED")
                return True
            if str(current).startswith(mode_upper) or str(current) == mode_upper:
                print(f"  {status_emoji(True)} 模式已切换为 {current}")
                self._hud_log_cmd(f"SET_MODE {mode_upper}", "ACCEPTED")
                return True

            time.sleep(0.2)

        print(f"  {status_emoji(False)} 模式切换超时")
        self._hud_log_cmd(f"SET_MODE {mode_upper}", "TIMEOUT")
        return False

    # ── Arm / Disarm ──────────────────────────────────

    def arm(self):
        """解锁"""
        print("[INFO] 发送 ARM 指令...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )

        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self._telemetry_lock:
                armed = self._shared_telemetry.get("armed")
            if armed:
                print(f"  {status_emoji(True)} ARM 成功")
                self._hud_log_cmd("ARM", "ACCEPTED")
                return True

            ack = self._drain_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if ack is False:
                print(f"  {status_emoji(False)} ARM 被拒绝 (检查安全开关/GPS/电池)")
                self._hud_log_cmd("ARM", "DENIED")
                return False
            time.sleep(0.2)

        print(f"  {status_emoji(False)} ARM 状态未确认")
        self._hud_log_cmd("ARM", "NO_ACK")
        return False

    def disarm(self):
        """上锁"""
        print("[INFO] 发送 DISARM 指令...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self._telemetry_lock:
                armed = self._shared_telemetry.get("armed")
            if armed is False:
                print(f"  {status_emoji(True)} DISARM 成功")
                self._hud_log_cmd("DISARM", "ACCEPTED")
                return True
            time.sleep(0.2)

        print(f"  {status_emoji(False)} DISARM 失败")
        self._hud_log_cmd("DISARM", "FAILED")
        return False

    # ── Takeoff (GUIDED 模式) ─────────────────────────

    def takeoff(self, altitude_m=3.0):
        """起飞到指定高度 (从共享遥测读取高度反馈)"""
        print(f"[INFO] Takeoff → {altitude_m:.1f} m...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, math.nan,
            0, 0, altitude_m,
        )

        deadline = time.time() + 30.0
        while time.time() < deadline:
            with self._telemetry_lock:
                alt = self._shared_telemetry.get("alt_rel")
            if alt is not None and alt >= altitude_m * 0.9:
                print(f"  {status_emoji(True)} 达到目标高度 {alt:.1f} m")
                self._hud_log_cmd(f"TAKEOFF {altitude_m:.0f}m", "ACCEPTED")
                return True
            if alt is not None:
                sys.stdout.write(f"\r  当前高度: {alt:.1f} m / {altitude_m:.1f} m  ")
                sys.stdout.flush()
            time.sleep(0.3)
        print(f"\n  {status_emoji(False)} Takeoff 超时")
        return False

    # ── Land / RTL ────────────────────────────────────

    def land(self):
        """降落"""
        print("[INFO] 发送 LAND 指令...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        print(f"  {status_emoji(True)} LAND 已发送")
        self._hud_log_cmd("LAND", "SENT")
        return True

    def rtl(self):
        """返航"""
        print("[INFO] 发送 RTL 指令...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        print(f"  {status_emoji(True)} RTL 已发送")
        self._hud_log_cmd("RTL", "SENT")
        return True

    # ── GUIDED 位置指令 ───────────────────────────────

    def goto_local_ned(self, x_m, y_m, z_m, yaw_rad=0):
        """GUIDED 模式: 飞到相对 NED 位置"""
        self.mav.mav.set_position_target_local_ned_send(
            0, 1, 1,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000110111111000,
            x_m, y_m, z_m, 0, 0, 0, 0, 0, 0, yaw_rad, 0,
        )
        desc = _ned_motion_name(x_m, y_m, z_m)
        self._hud_log_cmd(f"GOTO {desc} ({x_m:+.1f},{y_m:+.1f},{z_m:+.1f})", "SENT")
        return True

    # ── OFFBOARD 速度控制 ─────────────────────────────

    def send_offboard_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """发送 OFFBOARD 速度指令 (NED 坐标系, 需要 ≥2Hz 持续发送)"""
        self.mav.mav.set_position_target_local_ned_send(
            0, 1, 1,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate,
        )

    def offboard_velocity_ramp(self, vx, vy, vz, yaw_rate, duration_s,
                                ramp_time=0.3, freq_hz=15):
        """OFFBOARD 速度控制 (带缓启动/缓停)"""
        interval = 1.0 / freq_hz
        steps = int(duration_s / interval)
        ramp_steps = int(ramp_time / interval) if ramp_time > 0 else 1

        desc = _ned_motion_name(vx, vy, vz)
        print(f"[INFO] OFFBOARD 速度: {desc} vx={vx:.1f} vy={vy:.1f} vz={vz:.1f} "
              f"yaw_rate={yaw_rate:.2f}, 持续 {duration_s:.1f}s")

        cs = getattr(self, "_hud_cmd_state", None)
        if cs:
            cs["current_vel"] = {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate}
        start = time.time()

        for i in range(steps):
            if not self._running:
                break
            if i < ramp_steps:
                scale = (i + 1) / ramp_steps
            elif i >= steps - ramp_steps:
                scale = (steps - i) / ramp_steps
            else:
                scale = 1.0

            self.send_offboard_velocity(
                vx * scale, vy * scale, vz * scale, yaw_rate * scale)

            elapsed = time.time() - start
            sys.stdout.write(f"\r  速度 [{elapsed:.1f}s/{duration_s:.1f}s] "
                             f"vx={vx*scale:+.1f} vy={vy*scale:+.1f} "
                             f"vz={vz*scale:+.1f}  ")
            sys.stdout.flush()

            sleep_time = interval - (time.time() - start - i * interval)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.send_offboard_velocity(0, 0, 0, 0)
        if cs:
            cs["current_vel"] = {"vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0}
        print(f"\n  {status_emoji(True)} 速度控制完成 → 归零悬停")
        self._hud_log_cmd(f"{desc} {duration_s:.1f}s "
                          f"({vx:+.1f},{vy:+.1f},{vz:+.1f} m/s)", "DONE")

    # ── OFFBOARD 心跳保活 ─────────────────────────────

    def start_offboard_heartbeat(self):
        """启动后台 OFFBOARD 心跳 (20Hz 维持 OFFBOARD 模式)"""
        self._hb_running = True

        def _hb_loop():
            while self._hb_running:
                self.send_offboard_velocity(0, 0, 0, 0)
                time.sleep(0.05)

        self._hb_thread = threading.Thread(target=_hb_loop, daemon=True)
        self._hb_thread.start()
        print("[INFO] OFFBOARD 心跳已启动 (20Hz)")

    def stop_offboard_heartbeat(self):
        self._hb_running = False
        if hasattr(self, '_hb_thread'):
            self._hb_thread.join(timeout=1)


# ============================================================
# 自动测试序列
# ============================================================

def test_guided(ctrl: PX4Controller):
    """GUIDED 模式自动测试: arm → takeoff → move → land"""
    print_sep("GUIDED 模式测试")

    steps = [
        ("等待心跳",    ctrl.wait_for_heartbeat, []),
        ("查看状态",    ctrl.print_status, []),
        ("切 GUIDED",  ctrl.set_mode, ["GUIDED"]),
        ("ARM",        ctrl.arm, []),
        ("Takeoff 3m", ctrl.takeoff, [3.0]),
        ("悬停 3s",    time.sleep, [3.0]),
        ("前飞 2m",    ctrl.goto_local_ned, [2.0, 0.0, -3.0, 0]),
        ("等待到达",   time.sleep, [4.0]),
        ("右飞 2m",    ctrl.goto_local_ned, [0.0, 2.0, -3.0, 0]),
        ("等待到达",   time.sleep, [4.0]),
        ("LAND",       ctrl.land, []),
        ("等待降落",   time.sleep, [5.0]),
    ]

    for name, func, args in steps:
        print(f"\n── {name} ──")
        try:
            func(*args)
        except Exception as e:
            print(f"  {status_emoji(False)} 步骤失败: {e}")
            ctrl.land()
            return False
    return True


def test_offboard(ctrl: PX4Controller):
    """OFFBOARD 模式自动测试"""
    print_sep("OFFBOARD 模式测试")

    if not ctrl.set_mode("OFFBOARD"):
        return False
    ctrl.start_offboard_heartbeat()
    time.sleep(1)

    if not ctrl.arm():
        ctrl.stop_offboard_heartbeat()
        return False

    print(f"\n{'!' * 50}")
    print(f"  ⚠️  确认无人机已解锁且在地面/安全环境")
    print(f"{'!' * 50}\n")
    ans = input("  继续 OFFBOARD 测试? (y/N): ").strip().lower()
    if ans != 'y':
        print("  已取消")
        ctrl.disarm()
        ctrl.stop_offboard_heartbeat()
        return False

    maneuvers = [
        (0.0, 0.0, -0.5, 0.0, 3.0),
        (0.5, 0.0, 0.0, 0.0, 2.0),
        (-0.5, 0.0, 0.0, 0.0, 2.0),
        (0.0, 0.5, 0.0, 0.0, 2.0),
        (0.0, -0.5, 0.0, 0.0, 2.0),
    ]

    for vx, vy, vz, yr, dur in maneuvers:
        ctrl.offboard_velocity_ramp(vx, vy, vz, yr, dur, freq_hz=15)
        time.sleep(0.5)

    ctrl.offboard_velocity_ramp(0, 0, 0, 0, 1.0)
    ctrl.stop_offboard_heartbeat()
    ctrl.land()
    return True


# ============================================================
# 交互式菜单
# ============================================================

def interactive_menu(ctrl: PX4Controller):
    """交互式控制菜单"""
    if not ctrl.wait_for_heartbeat():
        print("[FAIL] 无法连接 PX4, 退出")
        return

    offboard_hb_active = False

    while ctrl._running:
        tele = ctrl.read_telemetry()
        armed = tele.get("armed", False)
        mode_name = tele.get("mode_name", "?")

        print("\n")
        print("┌" + "─" * 48 + "┐")
        print(f"│  PX4 飞行控制测试  │  {mode_name:<8}  "
              f"{'ARMED' if armed else 'DISARMED':<8}  │")
        print("├" + "─" * 48 + "┤")
        print(f"│  [1] 刷新状态                                  │")
        print(f"│  [2] 切换模式 (GUIDED/OFFBOARD/POSCTL/MANUAL/RTL)│")
        print(f"│  [3] {'Disarm' if armed else 'Arm'}                                          │")
        print(f"│                                               │")
        if not armed:
            print(f"│  [4] Takeoff (GUIDED 模式, 输入高度 m)         │")
        else:
            print(f"│  [5] Land                                       │")
            print(f"│  [6] RTL                                        │")
            print(f"│  [7] GUIDED goto (相对 NED: dx dy dz yaw°)     │")
            print(f"│  [8] OFFBOARD 速度 (vx vy vz yaw_rate 持续时间)│")
            print(f"│  [9] {'停止' if offboard_hb_active else '启动'} OFFBOARD 心跳                 │")
        print(f"│                                               │")
        print(f"│  [0] 退出                                      │")
        print("└" + "─" * 48 + "┘")

        try:
            choice = input("  → ").strip()
        except (EOFError, KeyboardInterrupt):
            choice = "0"

        if choice == "0":
            if armed:
                print("[WARN] 退出前 LAND...")
                ctrl.land()
            if offboard_hb_active:
                ctrl.stop_offboard_heartbeat()
            break

        elif choice == "1":
            ctrl.print_status()

        elif choice == "2":
            mode = input("  目标模式: ").strip().upper()
            if mode:
                if mode == "OFFBOARD" and not offboard_hb_active:
                    print("  [INFO] 自动启动 OFFBOARD 心跳")
                    ctrl.start_offboard_heartbeat()
                    offboard_hb_active = True
                ctrl.set_mode(mode)

        elif choice == "3":
            if armed:
                ctrl.disarm()
                if offboard_hb_active:
                    ctrl.stop_offboard_heartbeat()
                    offboard_hb_active = False
            else:
                print("  ⚠️  确认: 无人机在地面且安全区域")
                ans = input("  输入 YES 确认 ARM: ").strip()
                if ans == "YES":
                    ctrl.arm()

        elif choice == "4" and not armed:
            alt_s = input("  起飞高度 (m, 默认 3.0): ").strip()
            try:
                alt = float(alt_s) if alt_s else 3.0
            except ValueError:
                alt = 3.0
            if ctrl.set_mode("GUIDED"):
                time.sleep(0.5)
                ctrl.arm()
                time.sleep(0.5)
                ctrl.takeoff(alt)

        elif choice == "5" and armed:
            ctrl.land()
            if offboard_hb_active:
                ctrl.stop_offboard_heartbeat()
                offboard_hb_active = False

        elif choice == "6" and armed:
            ctrl.rtl()
            if offboard_hb_active:
                ctrl.stop_offboard_heartbeat()
                offboard_hb_active = False

        elif choice == "7" and armed:
            s = input("  输入 NED 偏移 (dx dy dz yaw_deg): ").strip()
            parts = s.split()
            if len(parts) >= 3:
                dx = float(parts[0])
                dy = float(parts[1])
                dz = float(parts[2])
                yaw = math.radians(float(parts[3])) if len(parts) >= 4 else 0
                ctrl.goto_local_ned(dx, dy, dz, yaw)

        elif choice == "8" and armed:
            s = input("  输入 vx vy vz yaw_rate dur_s: ").strip()
            parts = s.split()
            if len(parts) >= 4:
                vx = float(parts[0])
                vy = float(parts[1])
                vz = float(parts[2])
                yr = float(parts[3])
                dur = float(parts[4]) if len(parts) >= 5 else 2.0
                ctrl.offboard_velocity_ramp(vx, vy, vz, yr, dur)

        elif choice == "9":
            if offboard_hb_active:
                ctrl.stop_offboard_heartbeat()
                offboard_hb_active = False
                print("  OFFBOARD 心跳已停止")
            else:
                ctrl.start_offboard_heartbeat()
                offboard_hb_active = True

    print("\n  测试结束。")


# ============================================================
# 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="PX4 MAVLink 飞行控制测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                          # 交互式菜单 (默认)
  %(prog)s --mode guided            # 自动 GUIDED 测试
  %(prog)s --mode offboard          # 自动 OFFBOARD 测试
  %(prog)s --mode full              # 完整自动测试
  %(prog)s --hud                    # 带 HUD 屏幕显示
  %(prog)s --port /dev/ttyS3 --baud 115200
        """,
    )
    parser.add_argument("--port", "-p", default=DEFAULT_PORT,
                        help=f"串口设备 (默认: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD,
                        help=f"波特率 (默认: {DEFAULT_BAUD})")
    parser.add_argument("--mode", "-m", choices=["interactive", "guided", "offboard", "full"],
                        default="interactive",
                        help="测试模式 (默认: interactive=交互式菜单)")
    parser.add_argument("--hud", action="store_true",
                        help="启用 HDMI HUD 实时显示遥测")
    args = parser.parse_args()

    print_banner()
    print(f"  串口: {args.port} @ {args.baud}")
    print(f"  模式: {args.mode}")
    print(f"  HUD:  {'ON' if args.hud else 'OFF'}")

    ctrl = PX4Controller(args.port, args.baud)

    if not ctrl.connect():
        return 1

    # ── 启动后台遥测线程 (串口唯一读取者) ──
    ctrl.start_telemetry_thread()

    # ── HUD 初始化 ──
    hud = None
    hud_cmd_state = {
        "last_cmd": "-", "last_result": "-",
        "current_vel": {"vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0},
        "history": [],
    }
    hud_telemetry = {"connected": False}

    if args.hud:
        try:
            from hud_renderer import HUD
            hud = HUD()
            if hud.open():
                print("  [HUD] HDMI 显示已启动")
                hud._log("Control script started")

                def _hud_loop():
                    """HUD 刷新线程: 读共享缓存 → 渲染 → 写 fb (不碰串口)"""
                    tick = 1.0 / 10
                    next_tick = time.time()
                    while getattr(_hud_loop, "active", True):
                        now = time.time()
                        if now >= next_tick:
                            try:
                                with ctrl._telemetry_lock:
                                    hud_telemetry.update(ctrl._shared_telemetry)
                                hud_telemetry["connected"] = True
                                hud.update(hud_telemetry, hud_cmd_state)
                            except Exception:
                                pass
                            next_tick = now + tick
                        else:
                            time.sleep(0.01)

                _hud_loop.active = True
                hud_thread = threading.Thread(target=_hud_loop, daemon=True)
                hud_thread.start()
            else:
                print("  [HUD] HDMI 初始化失败, 跳过")
                hud = None
        except ImportError:
            print("  [HUD] hud_renderer 模块未找到, 跳过")
        except Exception as e:
            print(f"  [HUD] 初始化异常: {e}")

    # 安全提示
    print(f"\n  {'!' * 48}")
    print(f"  ⚠️  安全提醒:")
    print(f"  · 保持遥控器在手边，随时可以打杆接管")
    print(f"  · 确认无人机螺旋桨无遮挡")
    print(f"  · 首次测试建议桨叶不装或在地面进行")
    print(f"  · Ctrl+C 任何时候都会尝试 Land")
    print(f"  {'!' * 48}\n")

    # 信号处理: Ctrl+C → Land
    def emergency_shutdown(sig, frame):
        print("\n\n[EMERGENCY] Ctrl+C 触发紧急处理...")
        ctrl.send_offboard_velocity(0, 0, 0, 0)
        ctrl.land()
        ctrl.close()
        if hud:
            hud.close()
        sys.exit(0)

    import signal
    signal.signal(signal.SIGINT, emergency_shutdown)

    try:
        ctrl._hud_cmd_state = hud_cmd_state
        ctrl._hud = hud

        if args.mode == "interactive":
            interactive_menu(ctrl)
        elif args.mode == "guided":
            test_guided(ctrl)
        elif args.mode == "offboard":
            test_offboard(ctrl)
        elif args.mode == "full":
            if test_guided(ctrl):
                print("\n\n[INFO] GUIDED 测试完成, 进入 OFFBOARD 测试...")
                time.sleep(2)
                test_offboard(ctrl)
    finally:
        if hud:
            _hud_loop.active = False
            hud.close()
        ctrl.close()

    print("\n  脚本结束。\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
