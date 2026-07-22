# Zero3W — PX4 通信 & HUD 显示 开发状态文档

> 最后更新: 2026-07-22  
> 目标: 记录两个核心 Python 脚本的代码逻辑、功能边界和已知问题

---

## 一、文件清单与用途

| 文件 | 角色 | 可独立运行 | 可被 import |
|------|------|-----------|------------|
| `hud_renderer.py` | 屏幕渲染模块 | ✅ `sudo python3 hud_renderer.py` | ✅ `from hud_renderer import HUD` |
| `test_mavlink_control.py` | 飞行控制脚本 | ✅ `sudo python3 test_mavlink_control.py` | ❌ |
| `test_mavlink_serial.py` | 串口通信基础测试 | ✅ | ❌ |

---

## 二、hud_renderer.py

### 2.1 功能概述

在 HDMI 屏幕上用 OpenCV 渲染一个紧凑的实时遥测面板，类似 MSI Afterburner 的性能监控叠加层。

### 2.2 代码结构

```
436 行

├── 颜色常量 (BGR)                        第 30~48 行
│   └── 背景/面板/文字/OK/FAIL/WARN 等
│
├── HUD 类                               第 50~320 行
│   ├── __init__()                       初始化画布、缓冲、布局参数
│   ├── open()                           打开 /dev/fb0
│   │   ├── ioctl 检测实际分辨率/bpp/像素格式
│   │   ├── mmap framebuffer
│   │   └── 首帧全屏清黑
│   ├── close()                          关闭 fb
│   ├── _log(text)                       写日志 (deque, 最多 5 条)
│   ├── update(telemetry, cmd_state)     主渲染方法 (10Hz 调用)
│   │   ├── 收集遥测+指令数据
│   │   ├── 构建竖排列表 (每行 label: value)
│   │   │   TELEMETRY: Link/Mode/Armed/State/Roll/Pitch/Yaw/Alt/Vel/Bat/EKF
│   │   │   COMMANDS: Last Cmd/Result/Vel Cmd/History
│   │   │   LOG: 最近 5 条日志
│   │   │   底部: Frame # / Time
│   │   ├── 计算面板高度
│   │   ├── 画背景 + 边框
│   │   └── 画文字
│   └── _flip()                          面板区域 → cv2.cvtColor BGR→BGRA
│                                         → 逐行 os.lseek+os.write 写入 fb
│
├── 辅助函数
│   ├── _bgr_to_rgb565()                 16-bit 像素格式转换 (未使用)
│   └── _read_cpu_temp()                 读 CPU 温度 (未使用)
│
└── main()                               独立运行入口 (第 370~470 行)
    ├── 打开串口 (pymavlink) /dev/ttyS3 @115200
    ├── 等待 PX4 心跳
    ├── 线程 A: serial_loop()
    │   ├── 非阻塞读串口 (mav.port.in_waiting)
    │   ├── mav.mav.parse_buffer() 解析 MAVLink
    │   ├── 更新 tele dict (HEARTBEAT/ATTITUDE/LOCAL_POSITION_NED/...)
    │   ├── 检测 Mode/Armed 变化 → 写 LOG
    │   └── 每 5s 统计消息量 → 写 LOG
    └── 线程 B: hud_loop()
        └── 每 100ms 调用 hud.update(tele, cmd_state)
```

### 2.3 显示的面板字段

参见 [HUD_PARAMS.md](HUD_PARAMS.md)

### 2.4 HUD 类对外接口

```python
class HUD:
    def __init__(fb_device="/dev/fb0", width=1920, height=1080)
    def open() -> bool          # 打开 fb, 返回是否成功
    def close()                 # 清理
    def update(telemetry, cmd_state)  # 渲染一帧
    def _log(text)              # 写日志
```

**`update()` 的输入**:
- `telemetry` dict: 飞控遥测 (Link, Mode, Armed, State, Roll/Pitch/Yaw, Alt, Vel, Bat, EKF...)
- `cmd_state` dict: 控制指令状态 (last_cmd, last_result, current_vel, history)

### 2.5 已知问题

| 问题 | 状态 |
|------|------|
| OpenCV 内置字体不支持中文 | ✅ 已解决 — 全部标签改为英文 |
| 曾被 numpy `fb_arr[:] = bgra` 导致画面定期冻结 (~100帧) | ✅ 已解决 — 改用 os.lseek+os.write 逐行写入 |
| 曾被每行 tobytes() 导致画面出现小方框 | ✅ 已解决 — 改为整个面板一次 tobytes() 后逐行切片 |

---

## 三、test_mavlink_control.py

### 3.1 功能概述

通过 MAVLink 协议控制 PX4 飞控。支持交互式菜单和自动测试序列。

### 3.2 代码结构

```
~900 行

├── 常量                                   第 50~70 行
│   ├── PX4_MODE dict (模式名 → custom_mode 值)
│   └── MAVLink 标志位
│
├── 辅助函数
│   ├── _ned_motion_name(vx, vy, vz)      速度 → 人类可读描述
│   │   └── FWD/BACK/RIGHT/LEFT/UP/DOWN/HOVER
│   └── 其他显示函数
│
├── PX4Controller 类                      第 166~590 行
│   ├── __init__()                        串口参数、共享遥测状态、锁
│   ├── connect() / close()               串口连接
│   ├── wait_for_heartbeat(timeout)       等待 PX4 心跳
│   ├── read_telemetry(timeout)           批量读 MAVLink → 返回 tele dict
│   │   ├── HEARTBEAT  → armed, mode, state
│   │   ├── ATTITUDE   → roll/pitch/yaw + roll_deg/pitch_deg/yaw_deg
│   │   ├── LOCAL_POSITION_NED → vx, vy, vz
│   │   ├── GLOBAL_POSITION_INT → alt_rel
│   │   ├── BATTERY_STATUS → battery_v
│   │   ├── ESTIMATOR_STATUS → ekf_ok
│   │   └── 更新 _shared_telemetry (HUD 线程读取)
│   ├── print_status()                    打印当前飞控状态
│   ├── _hud_log_cmd(name, result)        更新 HUD 命令日志
│   ├── set_mode(mode_name)               模式切换 (GUIDED/OFFBOARD/RTL...)
│   ├── arm() / disarm()                  解锁/上锁
│   ├── takeoff(alt)                      起飞 (GUIDED 模式)
│   ├── land() / rtl()                    降落/返航
│   ├── goto_local_ned(x,y,z,yaw)         GUIDED 定点飞行
│   ├── send_offboard_velocity(vx,vy,vz,yr)  单次速度指令
│   ├── offboard_velocity_ramp(...)       OFFBOARD 速度控制 (带缓启动)
│   ├── start_offboard_heartbeat()        后台 20Hz 心跳保活
│   └── stop_offboard_heartbeat()
│
├── 自动测试序列
│   ├── test_guided(ctrl)                  GUIDED: arm→takeoff→goto→land
│   ├── test_offboard(ctrl)                OFFBOARD: 速度控制前后左右
│   └── interactive_menu(ctrl)             交互式菜单
│
└── main()                                入口
    ├── 解析命令行参数
    ├── --hud 模式: import HUD, 后台线程
    │   └── HUD 线程从 _shared_telemetry 读数据, 不碰串口
    ├── 信号处理 (Ctrl+C → Land)
    └── 根据 --mode 选择测试模式
```

### 3.3 控制能力清单

| 指令 | 方法 | MAVLink 消息 | 前提条件 |
|------|------|-------------|---------|
| 模式切换 | `set_mode()` | MAV_CMD_DO_SET_MODE | 飞控在 STBY 或 ACT |
| 解锁 | `arm()` | MAV_CMD_COMPONENT_ARM_DISARM | STBY 状态 |
| 上锁 | `disarm()` | MAV_CMD_COMPONENT_ARM_DISARM | 地面 |
| 起飞 | `takeoff(alt)` | MAV_CMD_NAV_TAKEOFF | GUIDED 模式 + Armed |
| 降落 | `land()` | MAV_CMD_NAV_LAND | 任意 |
| 返航 | `rtl()` | MAV_CMD_NAV_RETURN_TO_LAUNCH | 任意 |
| 定点飞行 | `goto_local_ned()` | SET_POSITION_TARGET_LOCAL_NED | GUIDED 模式 + Armed |
| 速度控制 | `offboard_velocity_ramp()` | SET_POSITION_TARGET_LOCAL_NED (≥2Hz) | OFFBOARD 模式 + Armed |

### 3.4 命令行用法

```bash
# 交互式菜单
sudo python3 test_mavlink_control.py

# 自动 GUIDED 测试
sudo python3 test_mavlink_control.py --mode guided

# 自动 OFFBOARD 测试
sudo python3 test_mavlink_control.py --mode offboard

# 完整自动测试
sudo python3 test_mavlink_control.py --mode full

# 带 HUD 显示 (有 bug, 见下文)
sudo python3 test_mavlink_control.py --hud

# 指定串口/波特率
sudo python3 test_mavlink_control.py --port /dev/ttyS3 --baud 115200
```

---

## 四、--hud 集成模式分析

### 4.1 设计思路

```
test_mavlink_control.py --hud
│
├── 主线程: 串口收发 + 控制逻辑
│   └── read_telemetry() → 更新 _shared_telemetry
│
└── HUD 线程: 每 100ms
    └── 读 _shared_telemetry → hud.update() → 写 /dev/fb0
```

### 4.2 已知问题：姿态显示为 0.0

**症状**: `--hud` 模式下 Roll/Pitch/Yaw 始终显示 0.0，但独立 `hud_renderer.py` 正常。

**根因**: 
1. `read_telemetry()` 原来只存 `roll/pitch/yaw` (弧度)，HUD 读 `roll_deg/pitch_deg/yaw_deg` (度) → 字段不匹配
2. `_shared_telemetry` 只在 `read_telemetry()` 被调用时更新。交互式菜单空闲时没有人调 `read_telemetry()` → HUD 读到的是旧数据

**第一次修复 (部分)**: 在 `read_telemetry()` 里同时输出 `roll_deg/pitch_deg/yaw_deg` (度)

**仍未解决**: `_shared_telemetry` 的更新时机问题。交互式菜单空闲时 HUD 数据停止更新。

**修复方向**: 
- 方案 A: 让控制脚本的主循环在空闲时也定期调用 `read_telemetry()`
- 方案 B: 让 HUD 线程直接用 `mav.recv_match()` 读串口（但会与主线程抢串口）
- 方案 C: 不用 `--hud` 集成。飞行时独立跑 `hud_renderer.py`，再加一个进程跑 `test_mavlink_control.py` 通过 localhost UDP 转发 MAVLink

**当前状态**: 暂未修复。独立 `hud_renderer.py` 工作正常，暂时够用。

---

## 五、当前项目文件索引

```
zero3w_test/zero_3w_mavlink/
├── test_mavlink_serial.py          # 串口基础通信测试 (✅ 稳定)
├── test_mavlink_control.py         # 飞行控制脚本 (✅ 控制功能可用)
├── hud_renderer.py                 # HUD 屏幕显示 (✅ 独立运行稳定)
├── HUD_PARAMS.md                   # HUD 每个参数的含义说明
├── DEBUG_LOG.md                    # DMA 冲突排查全记录 (面试用)
├── DISPLAY_AND_WIFI_PLAN.md        # WiFi 视频推流 + 网页仪表盘规划
└── DEV_STATUS.md                   # 本文档
```

---

## 六、下一步工作

| 优先级 | 任务 | 说明 |
|--------|------|------|
| P0 | 修复 `--hud` 集成模式 | 让控制+显示在一个进程里稳定运行 |
| P1 | 实机飞行测试 | 装桨、接 GPS、在 GUIDED 模式下飞 |
| P2 | 视觉识别集成 | 把 YOLO 气球检测 + IBVS 追踪接入控制 |
| P3 | WiFi MAVLink 转发 | Zero3W → UDP → QGC, 实现远程监控 |
| P4 | 视频推流 | MPP H.264 → UDP → QGC/浏览器 |

---

## 附录: 调试工具使用记录

```bash
# 进程状态
ps aux | grep python3          # D 状态 = 卡在内核 I/O

# 系统调用跟踪
strace -f -c -p PID            # 统计所有线程的系统调用
# 关键发现: write/lseek 每次仅 ~30µs, 不是瓶颈

# 性能分析
perf stat -p PID -- sleep 10   # CPU 周期、IPC、分支预测、缺页
perf record -p PID -g -- sleep 8 && perf report
# 关键发现: raw_array_assign_array 占 34% CPU (cvtColor 内部)

# 中断统计
cat /proc/interrupts | grep -E "dma|tty"
# 关键发现: DMA 中断计数为 0, 串口是中断模式
```
