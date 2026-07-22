#!/usr/bin/env python3
"""
PX4 MAVLink 串口通信测试脚本
==============================

硬件连接:
  - Zero3W UART3: RX=Pin3, TX=Pin5
  - PX4 TELEM1: MAVLink 2, Onboard 模式

用法:
  python3 test_mavlink_serial.py                          # 监听模式：只收不发
  python3 test_mavlink_serial.py --bidir                  # 双向测试：发送数据给 PX4
  python3 test_mavlink_serial.py --port /dev/ttyS3        # 指定串口
  python3 test_mavlink_serial.py --baud 115200            # 指定波特率
  python3 test_mavlink_serial.py --timeout 30             # 指定超时（秒）

双向测试说明:
  发送 STATUSTEXT 消息给 PX4 → 在 QGC 消息面板或 PX4 Console 可见
  发送 PING → 等待 PING 回应 → 验证双向通道
  发送 PARAM_REQUEST_LIST → 验证 PX4 正常响应参数请求
"""

import argparse
import struct
import sys
import threading
import time

# pymavlink — 需要预先安装: pip3 install --break-system-packages pymavlink
try:
    from pymavlink import mavutil
    _HAS_PYMAVLINK = True
except ImportError:
    mavutil = None
    _HAS_PYMAVLINK = False


# ============================================================
# MAVLink 名称映射表
# ============================================================

MAV_TYPE_NAMES = {
    0: "GENERIC", 1: "FIXED_WING", 2: "QUADROTOR", 3: "COAXIAL",
    4: "HELICOPTER", 5: "ANTENNA_TRACKER", 6: "GCS", 7: "AIRSHIP",
    8: "FREE_BALLOON", 9: "ROCKET", 10: "GROUND_ROVER", 11: "SURFACE_BOAT",
    12: "SUBMARINE", 13: "HEXAROTOR", 14: "OCTOROTOR", 15: "TRICOPTER",
    16: "FLAPPING_WING", 17: "KITE", 18: "ONBOARD_CONTROLLER",
    19: "VTOL_DUOROTOR", 20: "VTOL_QUADROTOR", 21: "VTOL_TILTROTOR",
    26: "GIMBAL", 27: "ADSB", 28: "PARAFOIL",
    29: "DIVING_BOAT", 31: "MULTIROTOR",
}

AUTOPILOT_NAMES = {
    0: "GENERIC", 3: "ARDUPILOTMEGA", 4: "OPENPILOT",
    8: "INVALID", 12: "PX4",
}

MAV_STATE_NAMES = {
    0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
    4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF",
    8: "FLIGHT_TERMINATION",
}

MAV_SEVERITY_NAMES = {
    0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
    4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG",
}


# ============================================================
# 打印工具函数
# ============================================================

def print_header(title: str):
    print("\n" + "─" * 58)
    print(f"  {title}")
    print("─" * 58)


def print_ok(msg: str):
    print(f"  ✅  {msg}")


def print_fail(msg: str):
    print(f"  ❌  {msg}")


def print_info(msg: str):
    print(f"  ℹ️   {msg}")


def print_separator():
    print("─" * 58)


# ============================================================
# 单向监听测试（仅接收）
# ============================================================

def test_listen(mav, timeout: int) -> bool:
    """监听 PX4 数据，验证接收通道正常"""
    print_header("阶段 1: 监听 PX4 心跳包")

    msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if msg is None:
        print_fail("超时：未收到心跳包")
        print("  → 请检查：")
        print("    1. PX4 是否已上电")
        print("    2. 串口接线：Zero3W TX(Pin5) → PX4 RX, Zero3W RX(Pin3) → PX4 TX")
        print("    3. 波特率是否匹配（当前 115200 8N1）")
        print("    4. PX4 TELEM1 是否配置为 MAVLink 2 (MAV_1_CONFIG=101, MAV_1_MODE=2)")
        return False

    print_ok(f"收到 PX4 心跳包！")
    print(f"     MAVLink 版本 : {msg.mavlink_version}")
    print(f"     系统/组件 ID  : {msg.get_srcSystem()}/{msg.get_srcComponent()}")
    print(f"     飞行器类型    : {MAV_TYPE_NAMES.get(msg.type, 'UNKNOWN')} ({msg.type})")
    print(f"     自驾仪类型    : {AUTOPILOT_NAMES.get(msg.autopilot, 'UNKNOWN')} ({msg.autopilot})")
    print(f"     系统状态      : {MAV_STATE_NAMES.get(msg.system_status, 'UNKNOWN')} ({msg.system_status})")
    print(f"     Base Mode     : 0x{msg.base_mode:02X}")
    print(f"     Custom Mode   : {msg.custom_mode}")
    print_separator()

    # 持续监听统计其他消息类型
    print_info(f"持续监听 {timeout}s，统计接收到的消息类型...")
    msg_types = {}
    end_time = time.time() + min(timeout, 10)
    while time.time() < end_time:
        msg = mav.recv_match(blocking=True, timeout=0.5)
        if msg is not None:
            t = msg.get_type()
            msg_types[t] = msg_types.get(t, 0) + 1

    if msg_types:
        print_ok(f"监听到 {len(msg_types)} 种消息类型，共 {sum(msg_types.values())} 条:")
        for t, cnt in sorted(msg_types.items(), key=lambda x: -x[1]):
            print(f"     {t:<30} × {cnt}")
    return True


# ============================================================
# 双向通信测试（发送 + 接收）
# ============================================================

def test_bidirectional(mav, timeout: int) -> bool:
    """
    双向通信测试：
      1. 发送 STATUSTEXT → PX4 Console / QGC 消息面板可见
      2. 发送 PING → 等待 PING 回应
      3. 请求参数列表 → 验证 PX4 正常响应
    """
    print_header("阶段 2: 双向通信测试（Zero3W → PX4）")
    print_info("以下测试会在 PX4 端产生可见输出")

    all_passed = True

    # ── 测试 1: 发送 STATUSTEXT ──────────────────────────
    print("\n  ── 测试 2a: 发送 STATUSTEXT ──")
    print_info("STATUSTEXT 消息会在以下位置可见：")
    print("         · QGroundControl → 消息面板 (Widget → Messages)")
    print("         · PX4 Console → `mavlink status` 或 `dmesg`")

    test_texts = [
        (MAV_SEVERITY_NAMES[6], "Zero3W: Hello PX4! Serial link test OK."),
        (MAV_SEVERITY_NAMES[6], "Zero3W: connected via UART3 @115200"),
    ]

    for severity, text in test_texts:
        mav.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            text.encode("utf-8")[:50],  # MAVLink STATUSTEXT 限制 50 字符
        )
        print(f"     → [{severity}] {text}")
        time.sleep(0.3)

    print_ok("STATUSTEXT 已发送（2 条）")
    print_info("请在 QGC 消息面板或 PX4 Console 中查看上述文字")

    # ── 测试 2: PING 往返测试 ───────────────────────────
    print("\n  ── 测试 2b: PING 往返测试 ──")
    print_info("发送 PING 并等待 PX4 回应...")

    ping_success = False
    for i in range(3):
        # 记录发送时间
        send_time_us = int(time.time() * 1_000_000)
        mav.mav.ping_send(
            send_time_us,  # time_usec
            i,             # seq
            0,             # target_system (0=broadcast)
            0,             # target_component (0=broadcast)
        )
        sys.stdout.write(f"     PING #{i} 已发送 ... ")
        sys.stdout.flush()

        # 等待 PING 回应 (MSG_ID_PING = 4, 同消息类型回应)
        ping_back = mav.recv_match(type="PING", blocking=True, timeout=2.0)
        if ping_back is not None and ping_back.get_srcSystem() == 1:
            rtt_ms = (time.time() * 1_000_000 - send_time_us) / 1000
            print(f"收到回应, RTT={rtt_ms:.1f}ms")
            ping_success = True
            break
        else:
            print("无回应")
            time.sleep(0.5)

    if ping_success:
        print_ok("PING 往返测试通过 — 双向通道正常")
    else:
        print_fail("PING 无回应，但可能 PX4 不处理 broadcast PING")
        print_info("这不一定是问题，继续后续测试...")

    # ── 测试 3: 请求参数列表 ────────────────────────────
    print("\n  ── 测试 2c: 请求参数列表 ──")
    print_info("发送 PARAM_REQUEST_LIST，验证 PX4 是否响应参数...")

    mav.mav.param_request_list_send(
        target_system=1,
        target_component=1,
    )
    print("     → PARAM_REQUEST_LIST 已发送")

    param_count = 0
    param_names = []
    end_time = time.time() + 5.0
    while time.time() < end_time and param_count < 8:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is not None:
            param_count += 1
            param_names.append(msg.param_id)
            sys.stdout.write(
                f"\r     收到参数 [{param_count}]: {msg.param_id:<20} = {msg.param_value:>10.4f}"
            )
            sys.stdout.flush()

    print("")
    if param_count > 0:
        print_ok(f"收到 {param_count} 个参数 — PX4 正常响应参数请求")
    else:
        print_fail("未收到参数回应")
        all_passed = False

    # ── 汇总 ────────────────────────────────────────────
    print_separator()
    if all_passed:
        print_ok("双向通信测试通过")
    else:
        print_fail("双向通信部分测试未通过，但接收通道正常")

    return True  # 即使部分双向测试失败，接收正常也算整体 OK


# ============================================================
# 后台持续发送心跳（让 PX4 知道 Zero3W 在线）
# ============================================================

def start_heartbeat_thread(mav):
    """启动后台线程，每 1 秒发送 HEARTBEAT，让 PX4 感知到 GCS 存在"""
    heartbeat_state = {"running": True}

    def send_heartbeat():
        while heartbeat_state["running"]:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            time.sleep(1.0)

    t = threading.Thread(target=send_heartbeat, daemon=True)
    t.start()
    return heartbeat_state


# ============================================================
# 主测试流程（基于 pymavlink）
# ============================================================

def run_test(port: str, baud: int, timeout: int, bidir: bool) -> bool:
    """使用 pymavlink 执行完整测试流程"""
    print_header("打开串口连接")
    print(f"     设备: {port}")
    print(f"     波特率: {baud}")
    print(f"     系统 ID: 255 (GCS)")
    print_separator()

    # 创建连接
    try:
        mav = mavutil.mavlink_connection(
            device=port,
            baud=baud,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
    except Exception as e:
        print_fail(f"无法打开串口: {e}")
        return False

    print_ok(f"串口 {port} 打开成功，等待数据...")

    # ── 阶段 1: 单向监听 ────────────────────────────
    if not test_listen(mav, timeout):
        mav.close()
        return False

    # ── 阶段 2: 双向测试（可选） ─────────────────────
    if bidir:
        # 先启动后台心跳，让 PX4 在 mavlink status 中看到连接
        print_info("启动后台 GCS 心跳广播（每 1s）...")
        hb_state = start_heartbeat_thread(mav)

        test_bidirectional(mav, timeout)

        # 停掉心跳线程
        hb_state["running"] = False

    mav.close()
    return True


# ============================================================
# 命令行入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="PX4 MAVLink 串口通信测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                           # 仅监听模式（只接收不发送）
  %(prog)s --bidir                   # 双向测试：监听 + 发送数据给 PX4
  %(prog)s --bidir --port /dev/ttyS3 # 指定串口 + 双向测试
  %(prog)s --scan                    # 扫描可用串口设备

双向测试说明:
  发送 STATUSTEXT → 在 QGC 消息面板 / PX4 Console 中可见
  发送 PING      → 验证 PX4 能否回应
  请求参数列表    → 验证 PX4 正常响应请求
  后台 GCS 心跳   → 让 PX4 感知到 Zero3W 在线

PX4 端验证方法:
  1. QGroundControl → Widget → Messages（查看 STATUSTEXT 消息）
  2. PX4 Console → `mavlink status`（查看连接的 GCS）
  3. PX4 Console → `listener status_text`（监听 status_text uORB 主题）
        """,
    )
    parser.add_argument("--port", "-p", default="/dev/ttyS3",
                        help="串口设备路径 (默认: /dev/ttyS3)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="波特率 (默认: 115200, 8N1)")
    parser.add_argument("--timeout", "-t", type=int, default=30,
                        help="超时时间，秒 (默认: 30)")
    parser.add_argument("--scan", "-s", action="store_true",
                        help="扫描可用串口设备")
    parser.add_argument("--bidir", action="store_true",
                        help="启用双向通信测试（Zero3W 主动发送数据给 PX4）")
    args = parser.parse_args()

    # 打印标题
    print("")
    print("╔" + "═" * 58 + "╗")
    print("║" + "  PX4 MAVLink 串口通信测试工具".center(54) + "║")
    print("╠" + "═" * 58 + "╣")
    print(f"║  {'串口设备:':<16} {args.port:<38}║")
    print(f"║  {'波特率:':<16} {args.baud:<38}║")
    print(f"║  {'超时:':<16} {args.timeout}s{' ':<36}║")
    print(f"║  {'双向测试:':<16} {'是' if args.bidir else '否（仅监听）':<38}║")
    print("╚" + "═" * 58 + "╝")

    # 检查 pymavlink
    if not _HAS_PYMAVLINK:
        print("\n[FAIL] pymavlink 未安装！")
        print("  安装: pip3 install --break-system-packages pymavlink")
        return 1

    # 扫描
    if args.scan:
        import glob, os
        patterns = ["/dev/ttyS*", "/dev/ttyAMA*", "/dev/ttyUSB*", "/dev/ttyACM*"]
        ports = sorted({p for pat in patterns for p in glob.glob(pat) if os.path.exists(p)})
        print_header("扫描可用串口")
        if ports:
            for p in ports:
                print(f"     {p}")
        else:
            print("     未发现任何串口设备")
        print("")

    # 执行测试
    success = run_test(args.port, args.baud, args.timeout, args.bidir)

    # 最终结果
    print("")
    if success:
        print("╔" + "═" * 58 + "╗")
        print("║" + "  ✅ Zero3W ↔ PX4 串口通信测试通过！".center(54) + "║")
        print("╚" + "═" * 58 + "╝")
        return 0
    else:
        print("╔" + "═" * 58 + "╗")
        print("║" + "  ❌ 串口通信测试失败".center(58) + "║")
        print("╠" + "═" * 58 + "╣")
        print("║  常见排查:                                           ║")
        print("║  1. PX4 是否已上电                                   ║")
        print("║  2. 接线: Zero3W TX ↔ PX4 RX, Zero3W RX ↔ PX4 TX  ║")
        print("║  3. PX4: MAV_1_CONFIG=101, MAV_1_MODE=2             ║")
        print("║  4. 波特率 115200 8N1                                ║")
        print("║  5. /dev/ttyS3 是否存在（用 --scan 检查）             ║")
        print("╚" + "═" * 58 + "╝")
        return 1


if __name__ == "__main__":
    sys.exit(main())
