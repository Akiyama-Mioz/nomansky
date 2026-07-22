# 无人机气球自主追踪系统 — 项目架构手册

> **目标**: Zero3W (RK3568) 机载计算机 → PX4 飞控 → 自主追踪气球  
> **语言**: C++11  
> **编译**: aarch64 交叉编译  
> **地面站**: QGroundControl (WiFi 局域网)  
> **最后更新**: 2026-07-21

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件拓扑与连接](#2-硬件拓扑与连接)
3. [总体架构：四层模型](#3-总体架构四层模型)
4. [三流分离：控制流 / 数据流 / 决策权](#4-三流分离控制流--数据流--决策权)
5. [目录结构](#5-目录结构)
6. [核心数据结构](#6-核心数据结构)
7. [线程架构](#7-线程架构)
8. [各层接口定义](#8-各层接口定义)
9. [控制层：状态机与运动规划](#9-控制层状态机与运动规划)
10. [视觉伺服控制：面积比 IBVS](#10-视觉伺服控制面积比-ibvs)
11. [OFFBOARD 模式处理](#11-offboard-模式处理)
12. [WiFi 视频推流](#12-wifi-视频推流)
13. [安全与错误处理](#13-安全与错误处理)
14. [线程安全设计](#14-线程安全设计)
15. [MAVLink C 库集成](#15-mavlink-c-库集成)
16. [交叉编译构建](#16-交叉编译构建)
17. [部署与运行](#17-部署与运行)
18. [实施计划](#18-实施计划)
19. [代码复用清单](#19-代码复用清单)
20. [关键设计决策记录](#20-关键设计决策记录)

---

## 1. 项目概述

### 1.1 项目目标

使用 Radxa Zero 3W 作为机载计算机，通过摄像头实时识别气球，自主控制无人机接近气球。

### 1.2 核心功能

| 功能 | 描述 |
|------|------|
| 气球识别 | YOLO (低频发现) + 颜色追踪 (高频伺服) 双模互补 |
| 视觉伺服 | IBVS 面积比控制: √(A₀/A) → 无需标定、无需知道气球尺寸 |
| 自主控制 | OFFBOARD 模式下自动追踪接近气球 |
| 视频回传 | H.264 硬编码 → WiFi UDP → QGC |
| 地面监控 | QGC 接收视频 + 遥测，可随时接管 |

> **视觉方案详见**: [docs/VISION_DESIGN.md](docs/VISION_DESIGN.md)

### 1.3 性能指标

| 指标 | 目标 |
|------|------|
| 视觉帧率 | ≥10Hz |
| 控制循环频率 | 20Hz |
| 视频推流帧率 | 15-20fps |
| 识别延迟 | <200ms (采集+推理+NMS) |
| 端到端延迟 | <300ms (识别到飞控响应) |

---

## 2. 硬件拓扑与连接

```
                          WiFi Router (192.168.66.x)
                               │
              ┌────────────────┼────────────────┐
              │                │                │
         ┌────┴────┐    ┌─────┴─────┐    ┌─────┴─────┐
         │  Zero3W  │    │    QGC    │    │  (Internet│
         │  (机载)   │    │  (地面站)  │    │  可选)    │
         └────┬────┘    └───────────┘    └───────────┘
              │
     UART3    │  USB
   ┌──────────┴──────────┐
   │                     │
┌──┴──────┐        ┌─────┴─────┐
│   PX4   │        │  Camera   │
│  飞控    │        │  (USB)    │
└─────────┘        └───────────┘
```

**连接详情：**

| 连接 | 接口 | 参数 |
|------|------|------|
| Zero3W ↔ PX4 | UART3 (Pin3=RX, Pin5=TX) | `/dev/ttyS3`, 115200 8N1, MAVLink v2 |
| Zero3W ↔ Camera | USB | `/dev/video0`, V4L2, NV12 |
| Zero3W ↔ QGC | WiFi | UDP 视频流 (端口 5600) |

---

## 3. 总体架构：四层模型

```
┌──────────────────────────────────────────────────────────────────────────┐
│                              Zero3W                                       │
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │                    决策层 (Decision Layer)                            │ │
│  │  MissionScheduler: 任务调度、模式仲裁、地面站指令处理                   │ │
│  │  频率: 5Hz | 职责: 决定"能不能飞"、"飞到哪里"                          │ │
│  └────────────────────────────┬────────────────────────────────────────┘ │
│                               │ MissionMode (IDLE/SEARCH/LOCK/TRACK/...) │
│  ┌────────────────────────────▼────────────────────────────────────────┐ │
│  │                    控制层 (Control Layer)                             │ │
│  │  StateMachine: SEARCH→LOCK→TRACK→HOVER→LOST 状态机                   │ │
│  │  MotionPlanner: 图像误差 → 速度指令 (IBVS 视觉伺服)                    │ │
│  │  频率: 20Hz | 职责: 决定"怎么飞"                                      │ │
│  └────────────────────────────┬────────────────────────────────────────┘ │
│                               │ TrackingResult (x_error, y_error, scale)  │
│  ┌────────────────────────────▼────────────────────────────────────────┐ │
│  │                     数据层 (Data Layer)                               │ │
│  │  VisualTracker: 双模调度 (SEARCH↔LOCK↔TRACK↔HOVER↔LOST)            │ │
│  │  YoloDetector: YOLO/RKNN 低频发现 (NPU, 0.5Hz)                       │ │
│  │  ColorTracker: 高频颜色追踪 (CPU, 30fps) + 面积比计算                  │ │
│  │  TelemetryReader: MAVLink 遥测解析 (姿态、位置、状态)                  │ │
│  │  职责: 纯计算，不做决策                                               │ │
│  │  详细设计: docs/VISION_DESIGN.md                                      │ │
│  └──────────────────┬──────────────────────┬───────────────────────────┘ │
│                     │ TrackingResult        │ Telemetry                   │
│  ┌──────────────────▼──────────────────────▼───────────────────────────┐ │
│  │                    通信层 (MAVLink Layer)                             │ │
│  │  MavlinkController: 发送 OFFBOARD 指令、心跳                          │ │
│  │  MavlinkReceiver: 读取串口 MAVLink 流、解析遥测                        │ │
│  │  职责: 纯传输，不处理数据                                            │ │
│  └──────────────────────────────┬──────────────────────────────────────┘ │
│                                 │ /dev/ttyS3, 115200 8N1                  │
└─────────────────────────────────┼──────────────────────────────────────────┘
                                  │
                             ┌────┴──────┐
                             │    PX4     │
                             │  飞控      │
                             └───────────┘
```

**关键原则**：上层调用下层，下层永远不知道上层的存在。每层只暴露最小接口。

---

## 4. 三流分离：控制流 / 数据流 / 决策权

### 4.1 控制流 (Control Flow) — 谁调用谁、什么频率

```
main()
  │
  ├── VisionThread (~10Hz SEARCH, 30fps TRACK)
  │     └─ SEARCH: 每 15 帧 YOLO → TRACK: 每帧颜色追踪+面积比
  │     └─ 写入 g_tracking_result
  │
  ├── MavlinkRxThread (事件驱动)
  │     └─ 阻塞读串口 → 逐字节喂 MAVLink 解析器 → 写入 g_telemetry
  │
  ├── MavlinkTxThread (20Hz 固定)
  │     └─ 读 g_motion_command → 打包 SET_POSITION_TARGET_LOCAL_NED → 写串口
  │     └─ 每秒一次 ONBOARD_CONTROLLER 心跳
  │
  ├── ControlThread (20Hz)  ← 主线程
  │     ├─ 1. 取最新 g_tracking_result (条件变量等待，超时 50ms)
  │     ├─ 2. 取最新 g_telemetry
  │     ├─ 3. 如果 tracking 有效 → 直接使用其中的 servo 指令
  │     ├─ 4. StateMachine::step(TrackingResult, Telemetry) → ControlCommand
  │     ├─ 5. 检查 g_mission_state (是否有决策层 override)
  │     ├─ 6. 写入 g_motion_command
  │     └─ 7. sleep_until 下一个 50ms tick
  │
  ├── VideoStreamThread (15-20fps)
  │     └─ 从三缓冲取最新帧 → MPP H.264 编码 → UDP sendto → QGC
  │
  └── SIGINT → g_running = false → join 所有线程 → 清理资源
```

### 4.2 数据流 (Data Flow) — 数据如何转换和传递

```
数据流 1: 视觉 → 控制 (IBVS)
═══════════════════════
[Camera /dev/video0]
    │ NV12 frame
    ▼
[VisualTracker] ───── 双模调度
    │ SEARCH: YOLO/RKNN (0.5Hz) → bbox
    │ LOCK:  自适应 HSV 采样 → 直方图 + A₀
    │ TRACK: 卡尔曼预测 → 局部反向投影 → 轮廓打分 → minEnclosingCircle
    │        面积比 √(A₀/A) → 视觉伺服速度指令
    ▼
[g_tracking_result] ── mutex + condition_variable
    │ TrackingResult {x_error, y_error, scale_factor, cmd_yaw, cmd_vx, cmd_vy, cmd_vz}
    ▼
[StateMachine] ─────── 阶段过渡判断 (SEARCH→TRACK→HOVER→LOST)
    │
    ▼
[g_motion_command] ─── atomic swap
    │
    ▼
[MavlinkTxThread] ──── 打包 MAVLink → /dev/ttyS3 → PX4


数据流 2: PX4 → Zero3W
═══════════════════════
[PX4] ── 持续输出 MAVLink 消息
    │ ATTITUDE, LOCAL_POSITION_NED, HEARTBEAT, BATTERY_STATUS...
    ▼
[/dev/ttyS3 读]
    │ 原始字节
    ▼
[MavlinkRxThread] ─── 逐字节 mavlink_parse_char()
    │ Telemetry {roll,yaw,alt,vx,vy,armed,mode,battery...}
    ▼
[g_telemetry] ──────── mutex
    │
    ├──→ [ControlThread] 用于位置估算 + 状态机判断
    └──→ [DecisionThread] 用于模式仲裁 + 安全检查


数据流 3: 视频 → 地面站
═══════════════════════
[Camera NV12 frame] (与视觉共用原始帧)
    │ 三缓冲拷贝
    ▼
[MPP Encoder] ──────── Rockchip 硬件 H.264 编码
    │ NAL units
    ▼
[UDP socket] ───────── sendto() → QGC 端口 5600


数据流 4: 地面站 → Zero3W (通过 PX4 转发)
═══════════════════════
[QGC] ── MAVLink COMMAND_LONG (LAND/RTL/LOITER)
    │ WiFi MAVLink UDP → PX4 → TELEM1 → Zero3W
    ▼
[MavlinkRxThread] ─── 解析 COMMAND_LONG
    │ GroundStationCmd
    ▼
[g_gs_cmd_queue] ───── mutex + queue
    │
    ▼
[DecisionThread] ───── 读取、仲裁、执行
```

### 4.3 决策权 (Decision Authority) — 谁有最终决定权

```
优先级 (从高到低):

  ┌────────────────────────────────────────────┐
  │ 1. 地面站紧急指令 (LAND / RTL)               │ ← 最高优先
  │    通过 QGC 发送，经由 PX4 MAVLink 转发       │    不可被自主控制覆盖
  ├────────────────────────────────────────────┤
  │ 2. PX4 飞控 FAILSAFE                        │ ← PX4 内部保护
  │    低电量、GPS 丢失、RC 丢失等                │    独立于 Zero3W
  ├────────────────────────────────────────────┤
  │ 3. 决策层安全约束                            │ ← Zero3W 决策层
  │    低电量检测、超时保护、地理围栏              │    可覆盖控制层输出
  ├────────────────────────────────────────────┤
  │ 4. 自主控制 (SEARCH / APPROACH / HOVER)     │ ← 正常模式
  │    控制层状态机输出的速度/位置指令            │
  ├────────────────────────────────────────────┤
  │ 5. 默认行为 (HOLD / 零速度悬停)              │ ← Fallback
  │    无检测、无遥测、任何异常情况               │
  └────────────────────────────────────────────┘
```

**核心原则**：
- **决策层拥有最终仲裁权**，控制层只负责"计算应该怎么飞"，不负责"决定能不能飞"
- 控制层输出的 `ControlCommand` 需要经过决策层检查后才能发给 MAVLink 层
- 地面站可以通过 PX4 随时发送 LAND/RTL 覆盖自主控制

---

## 5. 目录结构

```
zero3w_test/balloon_tracker/          # 项目根目录
│
├── PROJECT_MANUAL.md                 # 本手册 (AI Agent 入手指南)
├── CMakeLists.txt                    # 顶层 CMake
├── Makefile                          # 交叉编译 Makefile (直接编译用)
│
├── config/
│   └── default_config.yaml           # 配置文件
│
├── include/
│   ├── types.h                       # ⭐ 所有共享数据结构 (必读)
│   ├── constants.h                   # 系统常量 (sysid, compid, 频率等)
│   ├── decision/
│   │   └── mission_scheduler.h       # 决策层: 任务调度器
│   ├── control/
│   │   ├── state_machine.h           # 控制层: 状态机
│   │   └── motion_planner.h          # 控制层: 运动规划器
│   ├── data/
│   │   ├── vision_detector.h         # 数据层: 双模视觉追踪器
│   │   ├── color_tracker.h           # 数据层: 颜色追踪 (H-S直方图+轮廓)
│   │   ├── yolo_detector.h           # 数据层: YOLO/RKNN 检测器
│   │   ├── visual_servo.h            # 数据层: 视觉伺服 (面积比控制)
│   │   └── telemetry_reader.h        # 数据层: MAVLink 遥测解析
│   ├── mavlink/
│   │   ├── mavlink_serial.h          # 通信层: 串口 RAII 封装
│   │   ├── mavlink_controller.h      # 通信层: MAVLink 发送 (命令)
│   │   └── mavlink_receiver.h        # 通信层: MAVLink 接收 (遥测)
│   ├── video/
│   │   └── video_streamer.h          # 视频层: MPP 硬编码 + UDP 推流
│   └── utils/
│       ├── config.h                  # YAML 配置加载
│       ├── logger.h                  # 线程安全日志
│       └── circular_buffer.h         # 无锁环形缓冲
│
├── src/
│   ├── main.cpp                      # ⭐ 入口: 线程创建、信号处理、资源管理
│   ├── decision/
│   │   └── mission_scheduler.cpp
│   ├── control/
│   │   ├── state_machine.cpp
│   │   └── motion_planner.cpp
│   ├── data/
│   │   ├── vision_detector.cpp       # VisualTracker (双模调度)
│   │   ├── color_tracker.cpp         # 颜色追踪核心
│   │   ├── yolo_detector.cpp         # YOLO/RKNN
│   │   ├── visual_servo.cpp          # 视觉伺服控制
│   │   └── telemetry_reader.cpp
│   ├── mavlink/
│   │   ├── mavlink_serial.cpp
│   │   ├── mavlink_controller.cpp
│   │   └── mavlink_receiver.cpp
│   ├── video/
│   │   └── video_streamer.cpp
│   └── utils/
│       ├── config.cpp
│       └── logger.cpp
│
├── third_party/
│   └── mavlink_v2/                   # MAVLink C 库 (git submodule 或手动导入)
│       ├── common/                   # common MAVLink 消息定义
│       └── mavlink_types.h
│
├── model/
│   └── balloon_int8.rknn             # 气球检测 RKNN 模型
│
├── scripts/
│   ├── deploy.sh                     # 一键部署到 Zero3W
│   └── test_mavlink_serial.py        # (保留) Python 串口测试工具
│
└── docs/
    └── camera_calibration.md         # 相机标定文档
```

---

## 6. 核心数据结构

> 完整定义在 `include/types.h`。视觉相关的详细结构定义见 [docs/VISION_DESIGN.md §8](docs/VISION_DESIGN.md#8-关键数据结构)。
> 这里列出的是所有层共享的、经过精简的核心结构。视觉相关的 `BalloonDetection` 和 `RelativePosition` 已被 `TrackingResult` 替代。

```cpp
// ============================================================
// 视觉追踪结果 (替代原来的 BalloonDetection + RelativePosition)
// 由 VisualTracker 输出, 控制层直接使用
// ============================================================
struct TrackingResult {
    // 气球在图像中的状态
    float cx_px = 0.0f, cy_px = 0.0f;  // 最小外接圆中心 (像素)
    float radius_px = 0.0f;             // 最小外接圆半径 (像素)
    float A_current = 0.0f;             // 当前最小外接圆面积

    // 面积比 (核心: 替代绝对距离, 详见 VISION_DESIGN §11)
    float scale_factor = 1.0f;          // √(A₀/A), 1.0=参考距离

    // 图像空间误差 (归一化, 用于视觉伺服)
    float x_error = 0.0f;               // [-0.5, 0.5]
    float y_error = 0.0f;               // [-0.5, 0.5]

    // 视觉伺服速度指令 (已在追踪阶段计算)
    float cmd_yaw_rate = 0.0f;
    float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_vz = 0.0f;

    float best_score = 0.0f;            // 轮廓综合得分
    bool  valid = false;
    int   lost_counter = 0;
    std::chrono::steady_clock::time_point timestamp;
};

// ============================================================
// 视觉识别阶段
// ============================================================
enum class VisionPhase : uint8_t {
    SEARCH = 0,   // YOLO 低频搜索
    LOCK   = 1,   // 颜色采样 + 视觉居中
    TRACK  = 2,   // 高频颜色追踪 + 逼近
    HOVER  = 3,   // 悬停保持
    LOST   = 4,   // 丢失恢复
};

// ============================================================
// MAVLink 遥测 (从 PX4 汇总)
// ============================================================
struct Telemetry {
    // 姿态 (弧度)
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    // 本地 NED 位置 (米)
    float x_m = 0.0f, y_m = 0.0f, z_m = 0.0f;

    // NED 速度 (m/s)
    float vx_ms = 0.0f, vy_ms = 0.0f, vz_ms = 0.0f;

    // 全球位置
    double lat_deg = 0.0, lon_deg = 0.0;
    float alt_rel_m = 0.0f;     // 相对起飞点高度

    // 飞控状态
    bool armed = false;
    uint8_t base_mode = 0;
    uint32_t custom_mode = 0;
    bool offboard_active = false;

    // 电池
    float battery_v = 0.0f;
    int8_t battery_pct = -1;    // -1 = unknown

    bool valid = false;         // false = 从未收到过遥测
    std::chrono::steady_clock::time_point timestamp;
};

// ============================================================
// 控制指令 — 发给 MAVLink 层
// ============================================================
struct ControlCommand {
    enum Type { NONE = 0, VELOCITY, POSITION };

    Type type = NONE;

    // 速度模式 (NED 坐标系, m/s 和 rad/s)
    float vx_ms = 0.0f;
    float vy_ms = 0.0f;
    float vz_ms = 0.0f;         // 正=下降
    float yaw_rate_rads = 0.0f;

    // 位置模式 (NED, 米)
    float pos_x_m = 0.0f;
    float pos_y_m = 0.0f;
    float pos_z_m = 0.0f;
    float pos_yaw_rad = 0.0f;

    // 模式切换请求 (由决策层填入)
    bool request_arm = false;
    bool request_disarm = false;
    bool request_takeoff = false;
    float takeoff_alt_m = 5.0f;
    bool request_land = false;
    bool request_rtl = false;

    std::chrono::steady_clock::time_point timestamp;

    static ControlCommand hold() {
        ControlCommand cmd;
        cmd.type = VELOCITY;    // 零速度 = 悬停
        cmd.timestamp = std::chrono::steady_clock::now();
        return cmd;
    }
};

// ============================================================
// 决策层模式
// ============================================================
enum class MissionMode : uint8_t {
    IDLE = 0,       // 地面待命
    TAKEOFF,        // 起飞中
    SEARCH,         // 旋转搜索
    APPROACH,       // 接近气球
    HOVER,          // 气球附近悬停
    LOST,           // 丢失目标
    RTL,            // 返航
    LAND,           // 降落
    GCS_HOLD,       // 地面站暂停
    EMERGENCY,      // 紧急停止
};

// ============================================================
// 地面站指令 (经 PX4 MAVLink 转发)
// ============================================================
struct GroundStationCmd {
    enum Type { NONE = 0, LAND, RTL, LOITER, START_MISSION, STOP_MISSION };
    Type type = NONE;
    float param1 = 0.0f;
    std::chrono::steady_clock::time_point timestamp;
};

} // namespace balloon_chaser
```

---

## 7. 线程架构

### 7.1 线程总览

| 线程 | 频率 | CPU 占用 | 职责 |
|------|------|----------|------|
| **VisionThread** | 10Hz/30fps | 中 (NPU仅SEARCH) | 双模: YOLO(0.5Hz) + 颜色追踪(30fps) + 视觉伺服 |
| **MavlinkRxThread** | 事件驱动 | 低 | 阻塞读串口，逐字节解析 MAVLink |
| **MavlinkTxThread** | **20Hz 固定** | 低 | 发送 SET_POSITION_TARGET_LOCAL_NED + 心跳 |
| **ControlThread** | 20Hz | 低 | 读 TrackingResult → 状态机 → 写入指令变量 |
| **VideoStreamThread** | 15-20fps | 中 (MPP) | H.264 编码 + UDP 发送 |
| **DecisionThread** | 5Hz | 低 | 地面站指令处理、安全检查、模式仲裁 |

### 7.2 线程通信矩阵

| 共享变量 | 生产者 | 消费者 | 保护机制 | 说明 |
|----------|--------|--------|----------|------|
| `g_tracking_result` | VisionThread | ControlThread | `mutex` + `condvar` | 只保留最新一帧 |
| `g_telemetry` | MavlinkRxThread | ControlThread, DecisionThread | `mutex` | 整包替换 |
| `g_motion_command` | ControlThread | MavlinkTxThread | `std::atomic` 快照 | 只写最新指令 |
| `g_mission_state` | DecisionThread | ControlThread | `mutex` | 决策层输出 |
| `g_video_frame` | VisionThread | VideoStreamThread | 三缓冲 (triple buffer) | 不阻塞视觉 |
| `g_gs_cmd_queue` | MavlinkRxThread | DecisionThread | `mutex` + `queue` | FIFO 处理 |
| `g_running` | main (SIGINT) | 全部线程 | `std::atomic<bool>` | 优雅退出 |

### 7.3 为什么 MavlinkTxThread 必须是独立的 20Hz 线程

PX4 OFFBOARD 模式要求 **≥2Hz** 持续收到 `SET_POSITION_TARGET_LOCAL_NED`。如果 500ms 内没有任何指令，PX4 自动退出 OFFBOARD 回到 HOLD 模式。这是 PX4 的安全机制。

因此必须有一个独立线程以**严格 20Hz** 发送，即使控制循环暂时没有新指令，也要发送上一次的指令（或零速度 HOLD）。

---

## 8. 各层接口定义

### 8.1 通信层

```cpp
// mavlink_serial.h — 串口 RAII 封装
class MavlinkSerial {
public:
    bool open(const char* device, int baud);  // 打开 /dev/ttyS3
    void close();
    int  read_byte();                          // 阻塞读 1 字节, -1 = 超时/错误
    int  write_bytes(const uint8_t* data, size_t len);
    bool is_open() const;
};

// mavlink_controller.h — 发送 MAVLink 指令
class MavlinkController {
public:
    MavlinkController(MavlinkSerial& serial, uint8_t sysid, uint8_t compid);

    // OFFBOARD 核心 (由 MavlinkTxThread 以 20Hz 调用)
    void send_offboard_velocity(float vx, float vy, float vz, float yaw_rate);
    void send_offboard_position(float x, float y, float z, float yaw);

    // 模式控制 (由 DecisionThread 调用)
    void send_set_mode(uint8_t base_mode, uint32_t custom_mode);
    void send_arm_disarm(bool arm);
    void send_takeoff(float alt_m);
    void send_land();
    void send_rtl();

    // 心跳 (由 MavlinkTxThread 每秒调用)
    void send_heartbeat();

    // 调试 (由任意线程调用)
    void send_statustext(uint8_t severity, const char* text);
};

// mavlink_receiver.h — 接收 MAVLink 遥测
class MavlinkReceiver {
public:
    MavlinkReceiver(MavlinkSerial& serial);

    // 阻塞循环, 应放在 MavlinkRxThread 中
    void run();     // 不停读字节, 解析, 更新 g_telemetry
    void stop();

    // 线程安全读取 (由 ControlThread/DecisionThread 周期调用)
    Telemetry get_telemetry();

    // 地面站指令队列
    bool poll_gs_command(GroundStationCmd& cmd);
};
```

### 8.2 数据层

```cpp
// vision_detector.h — 双模视觉追踪器 (核心)
// 详细设计: docs/VISION_DESIGN.md
class VisualTracker {
public:
    bool init(const char* model_path, int camera_device);
    void shutdown();

    // 主更新 (每帧调用, 由 VisionThread 驱动)
    // 内部自动切换 SEARCH/LOCK/TRACK/HOVER/LOST 阶段
    VisionOutput process_frame();

    VisionPhase phase() const;
    void reset();  // 强制回到 SEARCH
};
```

// telemetry_reader.h — MAVLink 消息解析器
class TelemetryReader {
public:
    // 喂入一个字节, 内部累积并解析完整 MAVLink 消息
    // 当解析出完整消息时更新内部 Telemetry
    void feed_byte(uint8_t byte);

    // 获取最新遥测
    Telemetry latest() const;

    // 清空并返回地面站指令队列
    std::vector<GroundStationCmd> drain_gs_commands();
};
```

### 8.3 控制层

```cpp
// state_machine.h — 气球追踪状态机
enum class ApproachState : uint8_t {
    SEARCH   = 0,   // 搜索: 固定偏航角速度旋转
    APPROACH = 1,   // 接近: 比例控制追踪
    HOVER    = 2,   // 抵近: 零速度悬停
    LOST     = 3,   // 丢失: 短暂等待后回退
};

struct ApproachConfig {
    float search_yaw_rate_degs = 30.0f;    // 搜索旋转速度
    float approach_gain_xy = 0.5f;          // 水平比例增益
    float approach_gain_z = 0.3f;           // 垂直比例增益
    float hover_threshold_m = 2.0f;         // 进入 HOVER 的距离阈值
    float lost_timeout_s = 3.0f;            // 无检测 → LOST 的超时
    float max_approach_speed_ms = 5.0f;     // 接近速度上限
};

class StateMachine {
public:
    void configure(const ApproachConfig& cfg);
    ApproachState state() const;

    // 主更新函数: 每次 tick 返回下一个 ControlCommand
    ControlCommand tick(const TrackingResult& tracking,
                        const Telemetry& telemetry);
};

// motion_planner.h — 运动规划 (比例控制)
class MotionPlanner {
public:
    void configure(const ApproachConfig& cfg);

    // 由 StateMachine 在 APPROACH 状态中调用
    ControlCommand plan_approach(const TrackingResult& tracking);
    ControlCommand plan_search(const Telemetry& telemetry);

    // 紧急停止
    ControlCommand plan_hold();
};
```

### 8.4 决策层

```cpp
// mission_scheduler.h — 任务调度器
class MissionScheduler {
public:
    MissionMode mode() const;

    // 5Hz 调用: 读取遥测 + 地面站指令 + 状态机状态 → 决策模式切换
    void tick(const Telemetry& telemetry,
              const std::vector<GroundStationCmd>& gs_cmds,
              ApproachState approach_state,
              MavlinkController& mavlink);

    // 是否允许自主控制
    bool is_autonomous() const;

    // 决策层是否有 override (如 LAND/RTL)
    bool has_override(ControlCommand& override_cmd) const;
};
```

---

## 9. 控制层：状态机与运动规划

### 9.1 状态转移图

```
                         ┌──────────────────────────┐
                         │          IDLE             │
                         │  地面待命, 等待起飞        │
                         └──────────┬───────────────┘
                                    │ 决策层: arm + takeoff
                                    ▼
                         ┌──────────────────────────┐
              ┌─────────→│         SEARCH            │
              │          │  无检测: 固定 yaw_rate 旋转 │
              │          │  yaw_rate = 30°/s         │
              │          └──────────┬───────────────┘
              │                     │ detection.valid && conf > 0.5
              │                     ▼
              │          ┌──────────────────────────┐
              │          │        APPROACH           │
              │          │  vx = Kp_xy * north_m     │
              │          │  vy = Kp_xy * east_m      │
              │          │  vz = Kp_z  * down_m      │
              │          │  (clamp 到 max_speed)     │
              │          └────┬──────────┬──────────┘
              │               │          │
              │     distance  │          │ 连续 lost_timeout_s
              │     < hover_  │          │ 无检测
              │     threshold │          ▼
              │               │   ┌──────────────────┐
              │               │   │      LOST         │
              │               │   │ 悬停等待 recovery  │
              │               │   │ 超时 → SEARCH     │
              │               │   │ 恢复 → APPROACH   │
              │               │   └──────────────────┘
              │               ▼
              │        ┌──────────────────┐
              │        │      HOVER       │
              └────────│  零速度悬停       │
                       │  balloon远离→重新  │
                       │  进入 APPROACH    │
                       └──────────────────┘

任何状态收到 GCS LAND/RTL → 立即退出自主控制, 执行 LAND/RTL
```

### 9.2 状态转移条件表

| 当前状态 | 目标状态 | 条件 | 动作 |
|----------|----------|------|------|
| SEARCH | APPROACH | `detection.valid && conf >= 0.5` | 初始化比例控制器 |
| APPROACH | HOVER | `distance_m < hover_threshold_m` | 零速度悬停 |
| APPROACH | LOST | 连续 `lost_timeout_s` 无有效检测 | 悬停等待 |
| HOVER | APPROACH | `distance_m > hover_threshold_m * 1.5` | 重新追踪 |
| HOVER | LOST | 连续 `lost_timeout_s` 无有效检测 | 悬停等待 |
| LOST | APPROACH | 恢复有效检测 | 重新初始化 |
| LOST | SEARCH | `recovery_window_s` (如 10s) 超时 | 回到搜索旋转 |
| 任意 | LAND | GCS land 指令 | 降落 |
| 任意 | RTL | GCS rtl 指令 或 低电量 | 返航 |

### 9.3 比例控制公式 (APPROACH 状态)

```
vx = Kp_xy * rel_pos.north_m           -- 前向速度 (追赶)
vy = Kp_xy * rel_pos.east_m            -- 侧向速度 (对准)
vz = Kp_z  * rel_pos.down_m            -- 垂直速度 (高度对齐)
yaw_rate = 0                           -- 不动偏航 (靠 vy 侧移)

所有速度 clamp 到 [-max_approach_speed_ms, +max_approach_speed_ms]
```

使用 `SET_POSITION_TARGET_LOCAL_NED` 的 **速度控制模式** (type_mask 忽略位置字段)。

---

## 10. 视觉伺服控制：面积比 IBVS

> **这是本方案与传统 PBVS 方案的核心差异。完整推导和代码见 [VISION_DESIGN.md](docs/VISION_DESIGN.md)。**

### 10.1 核心理念

传统方案 (PBVS) 需要：相机标定 → 知晓气球尺寸 → 估算绝对距离 → NED 坐标 → 速度控制。

本方案 (IBVS)：**在图像空间中直接伺服**。不需要相机标定，不需要知道气球尺寸。

### 10.2 面积比替代绝对距离

```
小孔成像:  A ∝ 1/d²

参考帧: A₀ = k / d₀²
当前帧: A  = k / d²

相除消去 k:
    A₀/A = (d/d₀)²
→   scale_factor = √(A₀/A) = d/d₀

scale_factor = 1.0  →  在参考距离
scale_factor = 0.5  →  在参考距离的一半 (更近)
scale_factor = 2.0  →  在参考距离的两倍 (更远)
```

**d₀ (参考距离) 在比例中消去了。** 不需要知道它。

### 10.3 控制映射

```
x_error = (balloon_cx - image_cx) / image_width   // 水平偏差
y_error = (balloon_cy - image_cy) / image_height  // 垂直偏差

yaw_rate = K_YAW × x_error         // 左右转 → 水平居中
vz       = K_ALT × y_error         // 上下移 → 垂直居中
vx       = -K_VX × log(scale / target)  // 前后移 → 逼近目标距离
```

### 10.4 关键优势

| | 传统 PBVS (NED 坐标) | 本方案 IBVS (面积比) |
|---|---|---|
| 需要相机标定 | 是 (fx, fy, cx, cy) | **否** |
| 需要气球尺寸 | 是 (预估直径) | **否** |
| 需要地面假设 | 是 (距离估算) | **否** |
| 受姿态测量误差影响 | 是 (roll/pitch → NED) | **否** (纯图像空间) |
| 控制延迟 | 10Hz (YOLO) | **30fps** (颜色追踪) |

### 10.5 视觉识别五大阶段

详见 [VISION_DESIGN.md §2-7](docs/VISION_DESIGN.md)。

| 阶段 | 频率 | NPU | 作用 |
|------|------|-----|------|
| SEARCH | YOLO 0.5Hz | **有** | 发现气球 |
| LOCK | 一次性 ~2s | 无 | 建立颜色模型 + A₀ + 居中 |
| TRACK | 30fps | **无** | 高频追踪 + 面积比伺服 |
| HOVER | 30fps | 无 | 悬停保持 (零速补偿) |
| LOST | 30fps | 无 | 丢失恢复 / 回退 YOLO |

---

## 11. OFFBOARD 模式处理

### 11.1 PX4 OFFBOARD 合约

PX4 OFFBOARD 模式有一个硬性要求：
- **必须 ≥2Hz** 持续接收 `SET_POSITION_TARGET_LOCAL_NED` 消息
- 如果 **500ms** 内没有收到任何有效的 offboard 指令，PX4 **自动退出 OFFBOARD**，回退到 HOLD 模式

这是 PX4 的安全机制，防止机载计算机宕机后无人机失控。

### 11.2 我们的实现

```
MavlinkTxThread (独立线程, 严格 20Hz)
│
├─ 每 50ms:
│   ├─ 从 g_motion_command 读取最新指令 (atomic)
│   ├─ if 指令超过 500ms 未更新:
│   │    使用 HOLD (零速度) 指令 ← 保活 OFFBOARD
│   ├─ 打包 SET_POSITION_TARGET_LOCAL_NED
│   └─ 写入串口
│
├─ 每 1000ms (每 20 个 tick):
│   └─ 额外发送 HEARTBEAT (MAV_TYPE_ONBOARD_CONTROLLER)
│      → PX4 会在 mavlink status 中显示 Zero3W 的存在
```

### 11.3 OFFBOARD 进入序列

```
决策层执行:
1. send_set_mode(OFFBOARD)         // MAV_CMD_DO_SET_MODE
2. 等待 HEARTBEAT 确认 custom_mode == OFFBOARD
3. send_arm_disarm(true)           // 解锁
4. while (alt < takeoff_target):
      send_offboard_velocity(0, 0, -1.0, 0)  // 上升 1m/s
5. 进入 SEARCH 状态
```

### 11.4 OFFBOARD 退出序列

```
1. send_offboard_velocity(0, 0, 0, 0)  // 先悬停
2. send_land() 或 send_rtl()           // 切换模式
3. 等待 HEARTBEAT 确认模式已切换
```

---

## 12. WiFi 视频推流

### 12.1 方案

| 组件 | 选择 |
|------|------|
| 编码器 | Rockchip MPP 硬件 H.264 (RK3568 有 VPU) |
| 封装 | 裸 H.264 NAL units |
| 传输 | UDP 单播到 QGC |
| QGC 接收 | Application Settings → Video → UDP (端口 5600) |

### 12.2 数据路径

```
[Camera NV12 frame]
    │ 三缓冲拷贝 (从 VisionThread)
    ▼
[MPP Encoder]
    │ MPP H.264 编码 (硬件加速)
    │ ~5-10ms 编码延迟
    ▼
[NAL Unit Buffer]
    │ 每个 NAL unit ≤ MTU (1400 bytes)
    ▼
[UDP sendto() → QGC_IP:5600]
```

### 12.3 配置

- 分辨率: 640×480 (与检测分辨率解耦)
- 码率: 500kbps ~ 1Mbps (局域网带宽充裕)
- GOP: 30 (每秒一个关键帧)
- Profile: Baseline (低延迟)

---

## 13. 安全与错误处理

| 场景 | 检测方式 | 处理 | 日志级别 |
|------|----------|------|----------|
| 串口断开 | `g_telemetry.valid == false` 持续 >2s | 发送 HOLD，尝试重连 | ERROR |
| 摄像头断开 | V4L2 `poll()` 返回错误 | `detection.valid = false` → 状态机转 LOST | ERROR |
| NPU 推理失败 | `rknn_run()` 返回非零 | `detection.valid = false` | ERROR |
| OFFBOARD 超时退出 | HEARTBEAT 中 custom_mode 不再是 OFFBOARD | 尝试重新进入 | WARN |
| 低电量 | `battery_pct < 20` | 决策层强制 RTL | WARN |
| 检测过期 | detection timestamp age > 500ms | 发送 HOLD，等待恢复 | WARN |
| 遥测过期 | telemetry timestamp age > 500ms | 发送 HOLD | WARN |
| 地面站 LAND/RTL | `COMMAND_LONG` MAV_CMD | 最高优先级，立即执行 | INFO |
| 位置估算异常 | `distance > 100m` 或 `bearing.z > 0.5` | relative_pos.valid = false | DEBUG |

---

## 14. 线程安全设计

```
┌──────────────────────────────────────────────────────────┐
│  共享变量                保护机制              备注        │
├──────────────────────────────────────────────────────────┤
│  g_detection           mutex + condvar       Vision → Control │
│  g_telemetry           mutex                 Rx → Control, Decision │
│  g_motion_command      atomic swap           Control → Tx  │
│  g_mission_state       mutex                 Decision → Control │
│  g_video_frame [3]     per-slot mutex        Vision → Video │
│  g_gs_cmd_queue        mutex + std::queue    Rx → Decision │
│  g_running             atomic<bool>          全局优雅退出 │
└──────────────────────────────────────────────────────────┘
```

设计原则:
1. **最小化锁竞争**: 控制循环只持有锁的瞬间 (拷贝数据后立即释放)
2. **单向数据流**: 数据总是从生产者流向消费者，无环依赖
3. **只保留最新值**: 检测和遥测不排队 (队列深度 = 1)，控制指令也不排队
4. **视频帧用三缓冲**: 避免 VisionThread 等待编码完成

---

## 15. MAVLink C 库集成

### 15.1 获取 MAVLink C 库

```bash
cd third_party
git clone https://github.com/mavlink/c_library_v2.git mavlink_v2
# 或直接复制 minimal 头文件集
```

### 15.2 最小集成

只需要以下头文件:
```
mavlink_v2/
├── mavlink_types.h
├── mavlink_helpers.h
├── mavlink_conversions.h
├── protocol.h
└── common/           # 自动生成的消息编解码
    ├── mavlink.h
    ├── mavlink_msg_heartbeat.h
    ├── mavlink_msg_attitude.h
    ├── mavlink_msg_set_position_target_local_ned.h
    ├── mavlink_msg_command_long.h
    ├── mavlink_msg_statustext.h
    └── ...
```

### 15.3 交叉编译注意事项

MAVLink C 库是纯头文件的，不需要编译。但需要:
- 在编译时定义 `MAVLINK_USE_CONVENIENCE_FUNCTIONS`
- 设置正确的 `MAVLINK_MAX_PACKET_LEN` (默认 263 字节)
- 设置 `MAVLINK_COMM_NUM_BUFFERS` (默认 1)

在我们的 `CMakeLists.txt` 中:
```cmake
target_compile_definitions(balloon_tracker PRIVATE
    MAVLINK_USE_CONVENIENCE_FUNCTIONS
    MAVLINK_MAX_PACKET_LEN=263
)
```

---

## 16. 交叉编译构建

### 16.1 工具链

复用已有配置:
```
编译器: /usr/bin/aarch64-linux-gnu-g++ (Ubuntu 发行版)
或:     /opt/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++
```

### 16.2 CMakeLists.txt 结构

```cmake
cmake_minimum_required(VERSION 3.6)
project(balloon_tracker CXX)
set(CMAKE_CXX_STANDARD 11)

# 交叉编译
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 依赖路径
set(RKNN_INC /home/mio/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include)
set(RKNN_LIB /home/mio/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so)
set(OpenCV_DIR /home/mio/RKSDK/rknn-toolkit2/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)

# MAVLink 头文件
set(MAVLINK_INC ${CMAKE_SOURCE_DIR}/third_party/mavlink_v2)

find_package(OpenCV REQUIRED)
include_directories(
    ${RKNN_INC}
    ${MAVLINK_INC}
    ${CMAKE_SOURCE_DIR}/include
)

# 源文件
file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(balloon_tracker ${SOURCES})
target_link_libraries(balloon_tracker
    ${RKNN_LIB}
    ${OpenCV_LIBS}
    pthread dl rt
)

# 安装
install(TARGETS balloon_tracker DESTINATION ./)
install(PROGRAMS ${RKNN_LIB} DESTINATION lib)
```

### 16.3 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

---

## 17. 部署与运行

### 17.1 部署脚本

```bash
#!/bin/bash
# scripts/deploy.sh
BOARD_IP="192.168.66.223"
BOARD_USER="radxa"
BOARD_PASS="radxa"
TARGET_DIR="/home/radxa/balloon_tracker"

sshpass -p "$BOARD_PASS" ssh "$BOARD_USER@$BOARD_IP" "mkdir -p $TARGET_DIR"
sshpass -p "$BOARD_PASS" scp build/balloon_tracker "$BOARD_USER@$BOARD_IP:$TARGET_DIR/"
sshpass -p "$BOARD_PASS" scp build/librknnrt.so "$BOARD_USER@$BOARD_IP:$TARGET_DIR/"
sshpass -p "$BOARD_PASS" scp model/balloon_int8.rknn "$BOARD_USER@$BOARD_IP:$TARGET_DIR/"
sshpass -p "$BOARD_PASS" scp config/default_config.yaml "$BOARD_USER@$BOARD_IP:$TARGET_DIR/"
echo "Deploy complete."
```

### 17.2 运行

```bash
# SSH 到 Zero3W
ssh radxa@192.168.66.223

# 运行 (需要 root 权限访问串口)
cd /home/radxa/balloon_tracker
sudo LD_LIBRARY_PATH=. ./balloon_tracker
```

---

## 18. 实施计划

| 阶段 | 内容 | 依赖 | 验证方法 |
|------|------|------|----------|
| **1** | `types.h` + MAVLink C 库集成 + CMakeLists | 无 | 编译通过 |
| **2** | `mavlink_serial` + `mavlink_controller` (串口收发) | 1 | 类似 test_mavlink_serial.py, PX4 mavlink status 看到 GCS |
| **3** | `mavlink_receiver` + `telemetry_reader` (遥测解析) | 2 | 在终端打印 roll/yaw/alt 等遥测数据 |
| **4** | `color_tracker` + `visual_servo` (颜色追踪+面积比伺服) | 3 | 给定帧 → 验证追踪和伺服指令 |
| **5** | `state_machine` + `motion_planner` (状态机+IBVS控制) | 4 | 模拟 TrackingResult 输入, 验证状态转移和速度输出 |
| **6** | `mission_scheduler` + `main.cpp` (决策层+主入口) | 5 | 完整离地测试 (prop off) |
| **7** | `vision_detector` (集成已有 YOLO/RKNN 代码) | 4 | 验证检测结果格式正确 |
| **8** | `video_streamer` (MPP H.264 + UDP) | 无 | QGC 能看到视频 |
| **9** | 端到端集成 + 飞行测试 | 1-8 | 实际飞行, 从小距离开始 |

### 建议的并行工作
- 阶段 7 (视觉) 和阶段 8 (视频) 可与其他阶段并行开发
- 阶段 1-3 必须串行 (通信基础设施)
- 阶段 4-6 必须串行 (依赖链路)

---

## 19. 代码复用清单

| 现有代码 | 位置 | 复用方式 |
|----------|------|----------|
| YOLO 气球检测 (相机+NPU) | `zero3w_yolo_camera_demo/main.cc` | 抽取为 `YoloDetector` 类 (仅 SEARCH 阶段) |
| V4L2 相机采集 | `zero3w_test_camera/yolotest/` | 抽取为 `CameraCapture` 辅助 |
| YOLO 后处理 (NMS) | `zero_3w_yolo_test/postprocess.cc` | 直接复用, 改为单类 |
| **新增** | 颜色追踪 (H-S 直方图+反向投影) | 无现有代码, 全新 `color_tracker.cpp` |
| **新增** | 视觉伺服 (IBVS 面积比控制) | 无现有代码, 全新 `visual_servo.cpp` |
| Python MAVLink 测试 | `zero_3w_mavlink/test_mavlink_serial.py` | 参考消息类型/系统ID, C++ 复现 |
| 串口编程参考 | `uart_test/serial_chat.c` | 参考 termios 配置模式 |
| 交叉编译工具链 | `zero3w_yolo_camera_demo/Makefile` | 复用编译器路径/标志 |
| RKNN SDK | `/home/mio/RKSDK/rknn-toolkit2/rknpu2/` | 复用 include/lib |
| aarch64 OpenCV | RKSDK 中的 opencv-linux-aarch64 | 静态链接 |
| aarch64 sysroot | `zero_3w_yolo_test/sysroot/` | 交叉编译时需要 |

---

## 20. 关键设计决策记录

| 决策 | 选择 | 原因 |
|------|------|------|
| 控制模式 | OFFBOARD 速度控制 | 气球动态追踪不适合位置控制; 速度控制更平滑 |
| TX 线程独立 | 是 | PX4 OFFBOARD 需要 ≥2Hz 保活; 与控制循环解耦安全 |
| 线程数 | 6 (Vision/Rx/Tx/Control/Decision/Video) | 职责清晰分离; 都在 RK3568 4核能力范围内 |
| 视觉方案 | YOLO + 颜色追踪 IBVS (面积比) | 不需要相机标定、不需要气球尺寸; 详见 VISION_DESIGN |
| 状态机复杂度 | 5 阶段 (SEARCH/LOCK/TRACK/HOVER/LOST) | 双模切换 + 颜色采样 + 丢失恢复 |
| MAVLink 串口 | 115200 8N1 | 与 PX4 TELEM1 配置一致 |
| 决策层频率 | 5Hz | 模式仲裁不需要高频; 降低开销 |
| 视频编码 | Rockchip MPP 硬编码 | 零 CPU 开销, RK3568 有 VPU |

---

## 附录 A: PX4 参数配置清单

在 PX4 侧需要确认的参数 (通过 QGC 或 NSH 设置):

```
# TELEM1 端口配置
MAV_1_CONFIG     = 101    # TELEM 1
MAV_1_MODE       = 2      # Onboard mode
SER_TEL1_BAUD    = 115200 # 波特率匹配

# OFFBOARD 模式
COM_OBL_ACT      = 0      # Offboard 丢失后: 0=Land, 1=Hold, 2=Loiter
COM_OBL_RC_ACT   = 0      # Offboard+RC丢失: 0=Land

# 建议但非必须
COM_RCL_EXCEPT   = 4      # RC loss exception: OFFBOARD 不需要 RC
COM_RC_IN_MODE   = 1      # RC input mode
```

## 附录 B: 快速调试命令

在 Zero3W 上:
```bash
# 查看串口
ls -la /dev/ttyS*

# 查看摄像头
v4l2-ctl --list-devices

# 查看 NPU
cat /sys/kernel/debug/rknpu/load

# 查看 WiFi 连接
iwconfig wlan0
ip addr show wlan0

# 查看进程
ps aux | grep balloon_tracker
```

在 PX4 NSH Console 上:
```bash
# 查看 MAVLink 连接
mavlink status

# 查看飞控模式
commander status

# 查看 OFFBOARD 状态
listener vehicle_status
```
