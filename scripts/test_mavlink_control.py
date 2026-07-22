#!/usr/bin/env python3
"""
PX4 MAVLink 飞行控制测试脚本
==============================

功能: Zero3W 通过串口直接对 PX4 发送飞行控制指令。
      支持交互式菜单和自动测试序列。

硬件:  Zero3W UART3 (/dev/ttyS3) → PX4 TELEM1
协议:  MAVLink v2, 115200 8N1

用法:
  # 交互式菜单 (默认)
  python3 test_mavlink_control.py

  # 自动测试序列
  python3 test_mavlink_control.py --mode guided    # GUIDED 模式测试
  python3 test_mavlink_control.py --mode offboard  # OFFBOARD 模式测试
  python3 test_mavlink_control.py --mode full      # 完整测试 (GUIDED + OFFBOARD)

安全:
  - 遥控器保持开启作为紧急备份 (打杆自动退出 OFFBOARD)
  - Ctrl+C 任何时候都会尝试 Land + Disarm
  - 地面不会执行 ARM (需要确认)

依赖: pymavlink, pyserial
"""

import argparse
import math
import os
import struct
import sys
import threading
import time

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
    "GUIDED":       0x00040000,  # PX4 GUIDED = AUTO + 特定 sub-mode
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
    """从 PX4 custom_mode 提取主模式名。
    PX4 的 custom_mode 编码因版本而异, 尝试多种位偏移。"""
    # 尝试 bits 16-23 作为 main_mode (PX4 v1.13+ 常见)
    main_v1 = (custom_mode >> 16) & 0xFF
    sub_v1  = (custom_mode >> 24) & 0xFF

    # 也尝试 bits 0-7 (旧版 PX4)
    main_v0 = custom_mode & 0xFF
    sub_v0  = (custom_mode >> 8) & 0xFF

    names = {
        0: "MANUAL", 1: "ALTCTL", 2: "POSCTL",
        3: "AUTO",   4: "AUTO",   5: "LOITER",
        6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
    }
    auto_subs = {4: "LOITER", 5: "LOITER", 8: "RTL", 20: "TAKEOFF", 21: "LAND", 22: "FOLLOW"}

    # main_v1 非零且合理 → 用 v1 编码
    if 0 <= main_v1 <= 20:
        name = names.get(main_v1, f"MAIN{main_v1}")
        if main_v1 == 3 or main_v1 == 4:
            sub_name = auto_subs.get(sub_v1, f"SUB{sub_v1}")
            return f"AUTO.{sub_name}"
        return name

    # 回退到 v0
    name = names.get(main_v0, f"MAIN{main_v0}")
    if main_v0 == 3:
        sub_name = auto_subs.get(sub_v0, f"SUB{sub_v0}")
        return f"AUTO.{sub_name}"
    return name


def mav_state_name(state):
    return {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING",
            3: "STANDBY", 4: "ACTIVE", 5: "CRITICAL",
            6: "EMERGENCY", 7: "POWEROFF", 8: "FLIGHT_TERMINATION"}.get(state, f"?{state}")


# ============================================================
# 辅助函数
# ============================================================

def _ned_motion_name(vx, vy, vz):
    """NED 速度 → 人类可读的运动描述"""
    parts = []
    # 前后
    if vx > 0.1:   parts.append("FWD")
    elif vx < -0.1: parts.append("BACK")
    # 左右
    if vy > 0.1:   parts.append("RIGHT")
    elif vy < -0.1: parts.append("LEFT")
    # 上下
    if vz > 0.1:   parts.append("DOWN")
    elif vz < -0.1: parts.append("UP")
    # 悬停
    if not parts:
        parts.append("HOVER")
    return "+".join(parts)


# ============================================================
# PX4Controller 类
# ============================================================

class PX4Controller:
    """PX4 飞行控制器 — 封装所有 MAVLink 控制指令"""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.mav = None
        self._last_heartbeat = None
        self._last_attitude = None
        self._last_local_pos = None
        self._last_battery = None
        self._last_sys_status = None
        self._ack_queue = []
        self._ack_lock = threading.Lock()
        self._running = True
        # HUD 共享数据 (由主线程更新, HUD 线程只读)
        self._shared_telemetry = {"connected": False}
        self._telemetry_lock = threading.Lock()

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
        if self.mav:
            self.mav.close()

    # ── 心跳等待 ──────────────────────────────────────

    def wait_for_heartbeat(self, timeout=15):
        print(f"[INFO] 等待 PX4 心跳 (最多 {timeout}s)...")
        msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if msg is None:
            print(f"  {status_emoji(False)} 超时: 未收到心跳")
            return False
        print(f"  {status_emoji(True)} 已连接 PX4 (sysid={msg.get_srcSystem()}, "
              f"自驾仪=12(PX4), 类型=2(QUADROTOR))")
        return True

    # ── 遥测读取 ──────────────────────────────────────

    def read_telemetry(self, timeout=2.0):
        """读取所有可用遥测，返回 dict"""
        result = {
            "armed": None,
            "mode": None,
            "mode_name": "?",
            "state": None,
            "state_name": "?",
            "alt_rel": None,
            "vx": None, "vy": None, "vz": None,
            "battery_v": None,
            "roll": None, "pitch": None, "yaw": None,
        }
        end = time.time() + timeout
        while time.time() < end:
            msg = self.mav.recv_match(blocking=True, timeout=0.3)
            if msg is None:
                continue

            t = msg.get_type()

            if t == "HEARTBEAT":
                result["armed"] = (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
                result["mode"] = msg.custom_mode
                result["mode_name"] = px4_main_mode_name(msg.custom_mode)
                result["state"] = msg.system_status
                result["state_name"] = mav_state_name(msg.system_status)

            elif t == "ATTITUDE":
                result["roll"] = msg.roll
                result["pitch"] = msg.pitch
                result["yaw"] = msg.yaw
                result["roll_deg"] = math.degrees(msg.roll)
                result["pitch_deg"] = math.degrees(msg.pitch)
                result["yaw_deg"] = math.degrees(msg.yaw)

            elif t == "LOCAL_POSITION_NED":
                result["vx"] = msg.vx
                result["vy"] = msg.vy
                result["vz"] = msg.vz

            elif t == "GLOBAL_POSITION_INT":
                result["alt_rel"] = msg.relative_alt / 1000.0

            elif t == "BATTERY_STATUS":
                result["battery_v"] = msg.voltages[0] / 1000.0 if msg.voltages else None

            elif t == "COMMAND_ACK":
                with self._ack_lock:
                    self._ack_queue.append({
                        "command": msg.command,
                        "result": msg.result,
                    })

        # 更新共享遥测 (HUD 线程读取)
        with self._telemetry_lock:
            self._shared_telemetry.update(result)
        return result

    def print_status(self):
        """打印当前飞控状态"""
        tele = self.read_telemetry(timeout=1.5)
        print_sep("飞控状态")
        print(f"  Armed:         {'是' if tele['armed'] else '否'}")
        print(f"  模式:          {tele['mode_name']} (0x{tele['mode']:08X})"
              if tele['mode'] is not None else "  模式:          ?")
        print(f"  系统状态:      {tele['state_name']} ({tele['state']})"
              if tele['state'] is not None else "")
        if tele['alt_rel'] is not None:
            print(f"  相对高度:      {tele['alt_rel']:.1f} m")
        if tele['vx'] is not None:
            print(f"  速度 NED:      vx={tele['vx']:.1f}, vy={tele['vy']:.1f}, "
                  f"vz={tele['vz']:.1f} m/s")
        if tele['battery_v'] is not None:
            print(f"  电池:          {tele['battery_v']:.1f} V")
        if tele['roll'] is not None:
            print(f"  姿态 (r/p/y):  {math.degrees(tele['roll']):.0f}° "
                  f"{math.degrees(tele['pitch']):.0f}° "
                  f"{math.degrees(tele['yaw']):.0f}°")
        print_sep()
        return tele

    # ── 等待 COMMAND_ACK ──────────────────────────────

    def wait_ack(self, command_id, timeout=5.0):
        """等待特定 command 的 ACK，返回 True/False"""
        start = time.time()
        while time.time() - start < timeout:
            with self._ack_lock:
                for ack in self._ack_queue:
                    if ack["command"] == command_id:
                        ok = (ack["result"] == 0)  # MAV_RESULT_ACCEPTED
                        self._ack_queue.remove(ack)
                        return ok
                self._ack_queue.clear()  # 超时前清过期 ack
            time.sleep(0.1)
        return None  # 超时

    # ── 模式切换 ──────────────────────────────────────

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
        # 限制历史长度
        if len(cs["history"]) > 10:
            cs["history"] = cs["history"][-8:]
        # HUD 日志
        hud = getattr(self, "_hud", None)
        if hud:
            hud._log(f"TX: {cmd_name} -> {result}")

    def set_mode(self, mode_name):
        """切换到指定 PX4 模式，返回 True/False"""
        if mode_name.upper() == "GUIDED":
            # GUIDED 不是独立主模式, 用 MAV_CMD_NAV_TAKEOFF 隐式进入
            # 这里只检查是否已经是可接收 takeoff 的模式
            custom_mode = 0x00040004  # PX4: main=4 (AUTO), sub=4
        elif mode_name.upper() == "OFFBOARD":
            custom_mode = 0x00060006
        elif mode_name.upper() == "POSCTL":
            custom_mode = 0x00030002
        elif mode_name.upper() == "ALTCTL":
            custom_mode = 0x00010001
        elif mode_name.upper() == "STABILIZED":
            custom_mode = 0x000E0007
        elif mode_name.upper() == "MANUAL":
            custom_mode = 0x00000000
        elif mode_name.upper() == "RTL":
            custom_mode = 0x00080008
        elif mode_name.upper() == "LAND":
            custom_mode = 0x00150015
        elif mode_name.upper() == "LOITER":
            custom_mode = 0x00050005
        else:
            print(f"[FAIL] 未知模式: {mode_name}")
            return False

        print(f"[INFO] 切换模式 → {mode_name.upper()}...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode, 0, 0, 0, 0, 0,
        )

        # 等待 ACK 确认
        start = time.time()
        while time.time() - start < 5.0:
            msg = self.mav.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "COMMAND_ACK" and msg.command == 176:
                if msg.result == 0:
                    print(f"  {status_emoji(True)} SET_MODE ACCEPTED")
                    time.sleep(0.5)
                    return True
                else:
                    results = {1:"TEMP_REJECTED",2:"DENIED",3:"UNSUPPORTED",4:"FAILED"}
                    print(f"  {status_emoji(False)} SET_MODE {results.get(msg.result, '?')}")
                    return False
            elif msg.get_type() == "HEARTBEAT":
                # 同时检查 HEARTBEAT 中的模式是否已变化
                main_now = (msg.custom_mode >> 16) & 0xFF
                main_target = (custom_mode >> 16) & 0xFF
                if main_now == main_target and main_target != 0:
                    print(f"  {status_emoji(True)} 模式已切换为 {mode_name.upper()}")
                    self._hud_log_cmd(f"SET_MODE {mode_name.upper()}", "ACCEPTED")
                    return True

        print(f"  {status_emoji(False)} 模式切换超时")
        self._hud_log_cmd(f"SET_MODE {mode_name.upper()}", "FAILED")
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
        result = self.wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5.0)
        if result is None:
            # 有些固件不发 ACK，直接检查 HEARTBEAT
            pass
        time.sleep(1)
        tele = self.read_telemetry(timeout=1.0)
        if tele.get("armed"):
            print(f"  {status_emoji(True)} ARM 成功")
            self._hud_log_cmd("ARM", "ACCEPTED")
            return True
        elif result is False:
            print(f"  {status_emoji(False)} ARM 被拒绝 (检查安全开关/GPS/电池)")
            self._hud_log_cmd("ARM", "DENIED")
            return False
        else:
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
        time.sleep(1)
        tele = self.read_telemetry(timeout=1.0)
        if not tele.get("armed", True):
            print(f"  {status_emoji(True)} DISARM 成功")
            self._hud_log_cmd("DISARM", "ACCEPTED")
            return True
        print(f"  {status_emoji(False)} DISARM 失败")
        self._hud_log_cmd("DISARM", "FAILED")
        return False

    # ── Takeoff (GUIDED 模式) ─────────────────────────

    def takeoff(self, altitude_m=3.0):
        """起飞到指定高度 (需要 GUIDED 模式)"""
        print(f"[INFO] Takeoff → {altitude_m:.1f} m...")
        self.mav.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,                    # confirmation
            0, 0, 0, math.nan,    # param1-4: 忽略
            0, 0,                 # param5-6: lat, lon (0=当前位置)
            altitude_m,           # param7: 目标高度
        )

        # 等待达到高度
        start = time.time()
        while time.time() - start < 30.0:
            tele = self.read_telemetry(timeout=0.5)
            alt = tele.get("alt_rel")
            if alt is not None and alt >= altitude_m * 0.9:
                print(f"  {status_emoji(True)} 达到目标高度 {alt:.1f} m")
                self._hud_log_cmd(f"TAKEOFF {altitude_m:.0f}m", "ACCEPTED")
                return True
            if alt is not None:
                sys.stdout.write(f"\r  当前高度: {alt:.1f} m / {altitude_m:.1f} m  ")
                sys.stdout.flush()
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
        """GUIDED 模式: 飞到一个相对 NED 位置"""
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
            0,                    # time_boot_ms
            1, 1,                 # target_system, target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,   # type_mask: 只使用 VX,VY,VZ,YAW_RATE
            0, 0, 0,             # 位置 (忽略)
            vx, vy, vz,          # 速度
            0, 0, 0,             # 加速度 (忽略)
            0, yaw_rate,         # yaw (忽略), yaw_rate
        )

    def offboard_velocity_ramp(self, vx, vy, vz, yaw_rate, duration_s,
                                ramp_time=0.3, freq_hz=15):
        """
        OFFBOARD 速度控制 (带缓启动/缓停):
          vx, vy, vz:  目标速度 (m/s), NED 坐标系
          yaw_rate:    偏航角速度 (rad/s)
          duration_s:  持续时间 (秒)
          ramp_time:   加速/减速时间 (秒)
          freq_hz:     发送频率 (Hz)
        """
        interval = 1.0 / freq_hz
        steps = int(duration_s / interval)
        ramp_steps = int(ramp_time / interval) if ramp_time > 0 else 1

        print(f"[INFO] OFFBOARD 速度: vx={vx:.1f} vy={vy:.1f} vz={vz:.1f} "
              f"yaw_rate={yaw_rate:.2f}, 持续 {duration_s:.1f}s")
        # 更新 HUD 当前速度
        cs = getattr(self, "_hud_cmd_state", None)
        if cs:
            cs["current_vel"] = {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate}
        start = time.time()

        for i in range(steps):
            if not self._running:
                break

            # 缓启动/缓停
            if i < ramp_steps:
                scale = (i + 1) / ramp_steps  # 0 → 1
            elif i >= steps - ramp_steps:
                scale = (steps - i) / ramp_steps
            else:
                scale = 1.0

            self.send_offboard_velocity(
                vx * scale, vy * scale, vz * scale, yaw_rate * scale,
            )

            elapsed = time.time() - start
            sys.stdout.write(f"\r  速度 [{elapsed:.1f}s/{duration_s:.1f}s] "
                             f"vx={vx*scale:+.1f} vy={vy*scale:+.1f} "
                             f"vz={vz*scale:+.1f}  ")
            sys.stdout.flush()

            sleep_time = interval - (time.time() - start - i * interval)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.send_offboard_velocity(0, 0, 0, 0)  # 停止
        # 更新 HUD 速度显示为 0
        cs = getattr(self, "_hud_cmd_state", None)
        if cs:
            cs["current_vel"] = {"vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0}
        print(f"\n  {status_emoji(True)} 速度控制完成 → 归零悬停")
        desc = _ned_motion_name(vx, vy, vz)
        self._hud_log_cmd(f"{desc} {dur:.1f}s ({vx:+.1f},{vy:+.1f},{vz:+.1f} m/s)", "DONE")

    # ── OFFBOARD 心跳保活 ─────────────────────────────

    def start_offboard_heartbeat(self):
        """启动后台 OFFBOARD 心跳 (维持 OFFBOARD 模式)"""
        self._hb_running = True

        def _hb_loop():
            while self._hb_running:
                self.send_offboard_velocity(0, 0, 0, 0)  # 零速度 = 悬停
                time.sleep(0.05)  # 20Hz

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
        # (名称, 函数, 参数)
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
            # 紧急处理
            ctrl.land()
            return False
    return True


def test_offboard(ctrl: PX4Controller):
    """OFFBOARD 模式自动测试: 速度控制前后左右上下"""
    print_sep("OFFBOARD 模式测试")

    # 切模式 + 启动心跳
    if not ctrl.set_mode("OFFBOARD"):
        return False
    ctrl.start_offboard_heartbeat()
    time.sleep(1)

    if not ctrl.arm():
        ctrl.stop_offboard_heartbeat()
        return False

    # 只在地面做微小的速度测试 ← 安全!
    print(f"\n{'!' * 50}")
    print(f"  ⚠️  确认无人机已解锁且在地面/安全环境")
    print(f"{'!' * 50}\n")
    ans = input("  继续 OFFBOARD 测试? (y/N): ").strip().lower()
    if ans != 'y':
        print("  已取消")
        ctrl.disarm()
        ctrl.stop_offboard_heartbeat()
        return False

    # 序列: 上升 → 前进 → 后退 → 悬停 → Land
    maneuvers = [
        (0.0, 0.0, -0.5, 0.0, 3.0),   # 上升 0.5m/s × 3s
        (0.5, 0.0, 0.0, 0.0, 2.0),    # 前进 0.5m/s × 2s
        (-0.5, 0.0, 0.0, 0.0, 2.0),   # 后退 0.5m/s × 2s
        (0.0, 0.5, 0.0, 0.0, 2.0),    # 右移 0.5m/s × 2s
        (0.0, -0.5, 0.0, 0.0, 2.0),   # 左移 0.5m/s × 2s
    ]

    for vx, vy, vz, yr, dur in maneuvers:
        ctrl.offboard_velocity_ramp(vx, vy, vz, yr, dur, freq_hz=15)
        time.sleep(0.5)  # 机动间隙

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

    while True:
        tele = ctrl.read_telemetry(timeout=0.8)
        armed = tele.get("armed", False)
        mode_name = tele.get("mode_name", "?")

        print("\n" * 1)
        print("┌" + "─" * 48 + "┐")
        print(f"│  PX4 飞行控制测试  │  {mode_name:<8}  {'ARMED' if armed else 'DISARMED':<8}  │")
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
            ctrl.stop_offboard_heartbeat()
            offboard_hb_active = False

        elif choice == "6" and armed:
            ctrl.rtl()
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

                # 后台 HUD 刷新线程 (不读串口, 只渲染共享数据)
                def _hud_loop():
                    while getattr(_hud_loop, "active", True):
                        try:
                            # 用 mutex 保护读共享数据
                            with ctrl._telemetry_lock:
                                hud_telemetry.update(ctrl._shared_telemetry)
                            hud_telemetry["connected"] = True
                            hud.update(hud_telemetry, hud_cmd_state)
                        except Exception:
                            pass
                        time.sleep(0.1)  # 10Hz

                _hud_loop.active = True
                hud_thread = threading.Thread(target=_hud_loop, daemon=True)
                hud_thread.start()
            else:
                print("  [HUD] HDMI 初始化失败, 跳过")
                hud = None
        except ImportError:
            print("  [HUD] hud_renderer 模块未找到, 跳过")

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
        # 注入 HUD 到 PX4Controller (供 set_mode/arm 等方法更新命令状态)
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
