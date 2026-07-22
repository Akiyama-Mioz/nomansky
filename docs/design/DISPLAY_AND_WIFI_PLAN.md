# Zero3W 显示 & WiFi 通信 — 设计规划文档

> **目标**: 将 Zero3W ↔ PX4 的通信数据实时显示在屏幕上，并通过 WiFi 推流到地面站  
> **最后更新**: 2026-07-22

---

## 目录

1. [总体目标](#1-总体目标)
2. [通信架构升级](#2-通信架构升级)
3. [三端显示方案](#3-三端显示方案)
4. [HDMI 本地 HUD](#4-hdmi-本地-hud)
5. [WiFi 视频推流](#5-wifi-视频推流)
6. [MAVLink UDP 转发](#6-mavlink-udp-转发)
7. [网页仪表盘](#7-网页仪表盘)
8. [QGC 集成](#8-qgc-集成)
9. [Zero3W 进程架构](#9-zero3w-进程架构)
10. [数据流汇总](#10-数据流汇总)
11. [开发实施计划](#11-开发实施计划)

---

## 1. 总体目标

### 1.1 当前状态

```
Zero3W ──UART3──→ PX4      (控制指令 + 遥测, 双向)
Zero3W ──HDMI──→ 屏幕     (预留, 尚未开发)
Zero3W ──SSH───→ 电脑     (调试用)
```

### 1.2 目标状态

```
                          WiFi 局域网 (192.168.66.x)
              ┌────────────────┼────────────────┐
              │                │                │
         ┌────┴────┐    ┌─────┴─────┐    ┌─────┴─────┐
         │  Zero3W  │    │   电脑     │    │   QGC     │
         │  (机载)   │    │  (浏览器)  │    │  (地面站)  │
         └────┬────┘    └───────────┘    └───────────┘
              │
     UART3    │  USB
   ┌──────────┴──────────┐
   │                     │
┌──┴──────┐        ┌─────┴─────┐
│   PX4   │        │  Camera   │
└─────────┘        └───────────┘
```

三端各有不同用途：

| 端 | 显示内容 | 用途 |
|----|---------|------|
| **HDMI 屏幕** (本地) | 纯文本遥测 + 控制指令状态 | 快速开发调试, 不依赖网络 |
| **电脑浏览器** (WiFi) | 完整 HUD + 摄像头实时画面 | 视觉追踪调试, 自定义布局 |
| **QGC** (WiFi) | 飞行仪表 + 地图 + 视频窗口 | 正式飞行控制 |

---

## 2. 通信架构升级

### 2.1 当前: 纯串口

```
Zero3W ──UART3 (/dev/ttyS3, 115200, MAVLink v2)──→ PX4

只有一条物理链路, 所有 MAVLink 流量走这里。
```

### 2.2 升级后: 串口 + WiFi 双通道

```
                        ┌───────────────────────────┐
                        │         Zero3W              │
                        │                            │
PX4 ←──UART3──→ [mavlink_router] ←──UDP:14550──→ QGC │
                        │    ↕                       │
                        │ [web_server]                │
                        │    │  HTTP :8080            │
                        │    │  WebSocket :8081       │
                        │    ↓                        │
                        │ [video_streamer]            │
                        │    │  UDP :5600             │
                        └────┼───────────────────────┘
                             │
                        WiFi 局域网
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
         [QGC 地面站]   [浏览器仪表盘]   [其他设备]
         UDP:14550     HTTP:8080
         UDP:5600      视频解码
```

### 2.3 WiFi 数据通道总览

| 通道 | 方向 | 协议 | 端口 | 内容 |
|------|------|------|------|------|
| **MAVLink 遥测转发** | Zero3W → QGC/电脑 | MAVLink over UDP | 14550 | PX4 所有遥测 (HEARTBEAT, ATTITUDE, POSITION...) |
| **MAVLink 指令转发** | QGC → Zero3W → PX4 | MAVLink over UDP | 14550 | LAND, RTL, 模式切换等地面站指令 |
| **视频推流** | Zero3W → 电脑/QGC | UDP (H.264 裸流) | 5600 | 摄像头实时画面 |
| **Web 仪表盘** | Zero3W → 浏览器 | HTTP + WebSocket | 8080/8081 | 遥测 JSON + 指令状态 |
| **SSH 调试** | 电脑 → Zero3W | SSH | 22 | 运行脚本, 查看日志 |

---

## 3. 三端显示方案

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                  │
│   QGC (电脑)               网页仪表盘 (电脑浏览器)                │
│   ┌─────────────────┐     ┌──────────────────────────────┐     │
│   │  飞行仪表        │     │  自定义 HUD + 摄像头画面      │     │
│   │  地图            │     │  ┌──────────┬──────────┐    │     │
│   │  模式/状态       │     │  │ 遥测      │ 控制指令  │    │     │
│   │  视频窗口 (UDP)  │     │  │ 左栏      │ 右栏     │    │     │
│   └─────────────────┘     │  ├──────────┴──────────┤    │     │
│                           │  │    摄像头实时画面     │    │     │
│  用途: 飞行+地图          │  │    (640×480)         │    │     │
│                           │  ├─────────────────────┤    │     │
│                           │  │    日志             │    │     │
│                           │  └─────────────────────┘    │     │
│                           │                              │     │
│                           │  用途: 视觉追踪调试           │     │
│                           └──────────────────────────────┘     │
│                                                                  │
│   HDMI 屏幕 (本地 Zero3W)                                        │
│   ┌──────────────────────────────┐                              │
│   │  纯文本 HUD (不依赖网络)      │                              │
│   │  ┌──────────┬──────────┐    │                              │
│   │  │ 飞控遥测  │ 控制指令  │    │                              │
│   │  │ 左栏      │ 右栏     │    │                              │
│   │  ├──────────┴──────────┤    │                              │
│   │  │    日志             │    │                              │
│   │  └─────────────────────┘    │                              │
│   │                              │                              │
│   │  用途: 快速开发调试           │                              │
│   └──────────────────────────────┘                              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. HDMI 本地 HUD

### 4.1 布局

```
┌──────────────────────────────────────────────────────────────────┐
│                    PX4 MAVLink 实时监控                           │
│                    Zero3W ↔ PX4 通信状态                          │
│                    /dev/ttyS3 @ 115200                            │
├────────────────────────────┬─────────────────────────────────────┤
│     ← 飞控遥测 (PX4→Z3W)   │     控制指令 → (Z3W→PX4)            │
│                            │                                      │
│  【连接状态】               │  【最近指令】                         │
│  MAVLink: ✅               │  模式切换: GUIDED → ACCEPTED ✅     │
│  丢包率: 0.1%              │  ARM → TEMP_REJECTED ❌            │
│  RSSI:  ████████░░ 78%     │  Takeoff 3.0m → 等待中 ⏳          │
│                            │                                      │
│  【飞行状态】               │  【当前速度指令 (OFFBOARD)】          │
│  模式: AUTO.LOITER          │  vx: +1.0 m/s  ████████░░          │
│  Armed: 否                 │  vy:  0.0 m/s  ░░░░░░░░░░          │
│  系统状态: UNINIT           │  vz: -0.5 m/s  ████░░░░░░          │
│                            │  yaw_rate: 0.2 rad/s               │
│  【姿态】                   │                                      │
│  Roll:   -2°  ░░█░░░       │  【指令历史】 (最近 5 条)             │
│  Pitch:   1°  ░░░█░░       │  14:32:01 GUIDED → ACCEPTED        │
│  Yaw:     4°  █░░░░░       │  14:32:02 ARM → TEMP_REJECTED      │
│                            │  14:32:03 TAKEOFF 3m → 等待中       │
│  【位置 & 速度】            │                                      │
│  相对高度: 0.0 m           │                                      │
│  vx: 0.0   vy: 0.0        │                                      │
│  vz: 0.0   m/s            │                                      │
│                            │                                      │
│  【系统健康】               │                                      │
│  电池: N/A (USB供电)        │                                      │
│  GPS: N/A  卫星: 0         │                                      │
│  EKF: ✅  温度: 42°C       │                                      │
│                            │                                      │
├────────────────────────────┴─────────────────────────────────────┤
│  日志 (最新 5 条):                                                │
│  [14:32:01] SET_MODE GUIDED → ACCEPTED                            │
│  [14:32:02] ARM → TEMP_REJECTED (preflight)                       │
│  [14:32:03] RX: HEARTBEAT x12, ATTITUDE x168                      │
│  [14:32:04] TX: TAKEOFF 3.0m                                      │
│  [14:32:05] RX: COMMAND_ACK → REJECTED                            │
└──────────────────────────────────────────────────────────────────┘
```

### 4.2 数据刷新率

| 区块 | 刷新率 | 数据来源 |
|------|--------|----------|
| 连接状态 | 2Hz | HEARTBEAT 连续性 |
| 飞行状态 | 2Hz | HEARTBEAT |
| 姿态 | 10Hz | ATTITUDE |
| 位置/速度 | 10Hz | LOCAL_POSITION_NED + ALTITUDE |
| 系统健康 | 1Hz | SYS_STATUS, BATTERY_STATUS, ESTIMATOR_STATUS |
| 最近指令 | 事件驱动 | COMMAND_ACK |
| 指令历史 | 事件驱动 | 脚本自身记录 |
| 日志 | 事件驱动 | 全局日志队列 |

### 4.3 实现方式

```
OpenCV 渲染:
  · cv::Mat (1920×1080, BGRA) 作为画布
  · cv::putText()     → 文字
  · cv::rectangle()   → 柱状图、边框
  · cv::line()        → 分隔线
  · 写入 /dev/fb0 (Linux framebuffer)

渲染函数: render_hud(telemetry, cmd_state, log_lines) → cv::Mat
调用频率: 10Hz (每 100ms)
```

### 4.4 不需要显示的内容

- 原始 MAVLink hex 字节
- GPS 经纬度精确数值 (室内用不到)
- 所有 20 种消息类型的计数
- ATTITUDE_QUATERNION (有 roll/pitch/yaw 欧拉角就够了)

---

## 5. WiFi 视频推流

### 5.1 数据路径

```
[Camera USB /dev/video0]
    │ NV12, 640×480, 30fps
    ▼
[Zero3W 内存帧缓冲]
    │
    ├──→ [视觉追踪]  YOLO/颜色追踪 (用原始帧)
    │
    └──→ [MPP 硬编码]  Rockchip VPU, H.264 Baseline Profile
           │
           │ NAL units (每帧拆分为 ≤1400 bytes 的包)
           ▼
        [UDP Socket]
           │
           │ sendto("192.168.66.255:5600") 或单播到电脑 IP
           ▼
      WiFi 局域网
           │
    ┌──────┴──────┐
    ▼             ▼
  [QGC]       [网页仪表盘]
  UDP:5600    <video> 标签
  原生 H.264  或 Canvas + MSE
```

### 5.2 编码参数

| 参数 | 值 | 原因 |
|------|-----|------|
| 编码器 | Rockchip MPP (硬件 VPU) | 零 CPU, RK3568 内置 |
| 编码格式 | H.264 Baseline Profile | 低延迟, 兼容性最好 |
| 分辨率 | 640×480 | 追踪精度够, 带宽低 |
| 帧率 | 15-20fps | 视觉追踪只用到 ~10Hz |
| 码率 | 500kbps ~ 1Mbps | 局域网带宽充裕 |
| GOP | 30 (每秒一个 I 帧) | 花屏后 1s 内恢复 |
| 传输 | UDP 裸 H.264 NAL | QGC 原生支持 |

### 5.3 视频接收端方案

#### 方案 A: MJPEG over HTTP (开发阶段用)

```
Zero3W:  cv::imencode(".jpg", frame, buf)
         → HTTP multipart/x-mixed-replace 响应

网页:    <img src="http://192.168.66.223:8080/video">

优点: 一行 HTML, 零 JS
缺点: 每帧独立 JPEG, 带宽 ~2-5Mbps, 延迟较大
适用: 快速开发验证
```

#### 方案 B: H.264 over UDP → QGC (飞行时用)

```
Zero3W:  MPP 编码 → UDP sendto → 端口 5600

QGC:     Settings → Video → UDP Video Stream → 端口 5600
         自动解码显示

优点: 低延迟, QGC 原生支持, 带宽 ~500kbps
缺点: 浏览器不能直接播放
```

#### 方案 C: H.264 → WebSocket → 浏览器 MSE (推荐最终方案)

```
Zero3W:  MPP 编码 → fMP4 封装 → WebSocket push

浏览器:  MediaSource API → <video> 播放

优点: 低延迟 + 浏览器原生 <video> 标签
缺点: 需要 JS 代码封装 MSE stream
适用: 最终版本
```

### 5.4 推荐路线

```
开发阶段:   方案 A (MJPEG) — 先出画面, 一天搞定
飞行测试:   方案 B (UDP + QGC) — QGC 原生支持, 零额外开发
最终版本:   方案 C (MSE + WebSocket) — 低延迟 + 浏览器播放
```

---

## 6. MAVLink UDP 转发

### 6.1 Zero3W 作为 MAVLink 路由器

```
         串口 (/dev/ttyS3)                UDP (:14550)
         ←───────────────                ←───────────────
PX4 ─────→ [mavlink_router.py] ──────────→ QGC / 电脑
              │
              │ 旁路: 所有消息也推送到 WebSocket (给网页仪表盘)
              ▼
         WebSocket (:8081)
```

### 6.2 转发逻辑

```python
# mavlink_router.py 伪代码

serial = open("/dev/ttyS3", 115200)
udp_sock = socket(AF_INET, SOCK_DGRAM)
udp_sock.bind(("0.0.0.0", 14550))

# GCS 地址 (可配置)
gcs_addr = ("192.168.66.255", 14550)  # 广播 或 单播

while running:
    # 1. 从串口读 PX4 消息 → 转发到 UDP
    if serial.in_waiting:
        data = serial.read(serial.in_waiting)
        udp_sock.sendto(data, gcs_addr)

    # 2. 从 UDP 读 GCS 消息 → 转发到串口
    data, addr = udp_sock.recvfrom(4096)
    serial.write(data)
    gcs_addr = addr  # 记住最后发来消息的 GCS 地址
```

### 6.3 QGC 配置

```
QGC → Application Settings → Comm Links
  → 添加 → UDP
  → 监听端口: 14550
  → 目标主机: 192.168.66.223 (Zero3W IP)
```

---

## 7. 网页仪表盘

### 7.1 技术栈

| 层 | 选择 | 原因 |
|----|------|------|
| HTTP 服务器 | Python `http.server` + `socketserver` | 简单, 零依赖 |
| WebSocket | Python `websockets` 库 | 双向推送遥测 |
| 前端 | 纯 HTML + JS + Canvas | 不需要构建工具 |
| 视频 | MJPEG (阶段1) → MSE (阶段2) | 见 §5.3 |

### 7.2 网页布局

```
┌──────────────────────────────────────────────────────────────────┐
│                PX4 MAVLink 实时监控 + 视觉追踪                     │
│                Zero3W ↔ PX4  |  WiFi → 192.168.66.223:8080        │
├─────────────────────────────┬────────────────────────────────────┤
│   ← 飞控遥测 (PX4→Z3W)      │   控制指令 → (Z3W→PX4)              │
│                             │                                     │
│  【连接】                   │  【最近指令】                        │
│  MAVLink: ✅  WiFi: ✅     │  切 GUIDED → ACCEPTED ✅            │
│                             │  ARM → TEMP_REJECTED ❌            │
│  【飞行状态】                │                                     │
│  模式: AUTO.LOITER          │  【当前速度指令】                    │
│  Armed: 否  状态: UNINIT    │  vx: +1.0  vy: 0.0                 │
│                             │  vz: -0.5  yr: 0.2                │
│  【姿态】                   │                                     │
│  R: -2  P: 1  Y: 4        │  【指令历史】 (最近 5 条)            │
│  【位置速度】               │  14:32:01 GUIDED → OK              │
│  H: 0.0m                   │  14:32:02 ARM → REJECTED           │
│  【健康】 电池 N/A  EKF ✅  │                                     │
│                             │                                     │
├─────────────────────────────┴────────────────────────────────────┤
│                                                                   │
│                    📷 摄像头实时画面                                │
│               (640×480, MJPEG 或 H.264 解码)                      │
│                                                                   │
│    ┌──────────────────────────────────────────────────┐          │
│    │                                                   │          │
│    │              [气球检测框 + 追踪轮廓]                │          │
│    │              (后期叠加)                            │          │
│    │                                                   │          │
│    └──────────────────────────────────────────────────┘          │
│                                                                   │
│  视觉状态: 检测 🎈 conf=0.87  |  追踪 ✅ scale=0.52  score=0.91    │
│                                                                   │
├───────────────────────────────────────────────────────────────────┤
│  日志:  [14:32:01] SET_MODE GUIDED → ACCEPTED                     │
│         [14:32:02] 收到 HEARTBEAT x12, ATTITUDE x168               │
└───────────────────────────────────────────────────────────────────┘
```

### 7.3 WebSocket 推送数据格式 (JSON)

```json
{
  "timestamp": "14:32:01.234",
  "telemetry": {
    "armed": false,
    "mode_name": "AUTO.LOITER",
    "mode_raw": "0x04040000",
    "system_status": 0,
    "state_name": "UNINIT",
    "roll_deg": -2.3,
    "pitch_deg": 1.1,
    "yaw_deg": 4.2,
    "alt_rel_m": 0.0,
    "vx_ms": 0.0, "vy_ms": 0.0, "vz_ms": 0.0,
    "battery_v": null,
    "ekf_ok": true
  },
  "command": {
    "last_cmd": "ARM",
    "last_result": "TEMP_REJECTED",
    "current_velocity": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0},
    "history": [
      {"time": "14:32:01", "cmd": "GUIDED", "result": "ACCEPTED"},
      {"time": "14:32:02", "cmd": "ARM", "result": "TEMP_REJECTED"}
    ]
  },
  "vision": {
    "detected": true,
    "confidence": 0.87,
    "tracking": true,
    "scale_factor": 0.52,
    "score": 0.91
  },
  "logs": [
    "[14:32:01] SET_MODE GUIDED → ACCEPTED",
    "[14:32:02] ARM → TEMP_REJECTED"
  ]
}
```

---

## 8. QGC 集成

### 8.1 QGC 能接收什么

| 数据类型 | 方式 | QGC 显示位置 |
|----------|------|-------------|
| 飞控遥测 | MAVLink UDP (:14550) | 飞行仪表、地图、状态栏 |
| 视频流 | UDP H.264 (:5600) | 视频窗口 (需在 Video Settings 中配置) |
| STATUSTEXT | MAVLink 消息 | Messages 面板 |

### 8.2 QGC 配置步骤

```
1. Comm Links → 添加 UDP → 端口 14550 → 连接
2. Application Settings → Video:
   - Source: UDP Video Stream
   - Port: 5600
3. Widget → Video 打开视频窗口
```

---

## 9. Zero3W 进程架构

### 9.1 当前 (单一脚本)

```
└── test_mavlink_control.py
       ├── 串口收发
       ├── 控制逻辑
       └── 终端打印
```

### 9.2 升级后 (多进程)

```
┌─────────────────────────────────────────────────────────┐
│                      Zero3W 进程                         │
│                                                          │
│  ┌──────────────────┐   ┌──────────────────┐            │
│  │ mavlink_router   │   │ web_dashboard    │            │
│  │ (Python)         │   │ (Python)         │            │
│  │                  │   │                  │            │
│  │ 串口↔UDP 双向    │   │ HTTP :8080       │            │
│  │ 转发 MAVLink     │   │ WebSocket :8081  │            │
│  │ 端口: UDP 14550  │   │ 推送遥测 JSON     │            │
│  └────────┬─────────┘   └────────┬─────────┘            │
│           │                      │                      │
│           │    ┌─────────────────┘                      │
│           │    │                                        │
│           ▼    ▼                                        │
│  ┌──────────────────┐   ┌──────────────────┐            │
│  │ telemetry_state  │   │ static_files/    │            │
│  │ (共享内存/文件)   │   │ index.html       │            │
│  │ 最新遥测缓存      │   │ dashboard.js     │            │
│  └────────┬─────────┘   │ style.css        │            │
│           │             └──────────────────┘            │
│           │                                            │
│  ┌────────┴─────────┐                                  │
│  │ video_streamer   │                                  │
│  │ (C++ 或 Python)  │                                  │
│  │                  │                                  │
│  │ 摄像头采集        │                                  │
│  │ MPP H.264 编码   │                                  │
│  │ UDP :5600 推流   │                                  │
│  └──────────────────┘                                  │
│                                                          │
│  ┌──────────────────┐                                   │
│  │ test_mavlink_    │  (保留, 调试用)                    │
│  │ control.py       │                                   │
│  │ + HDMI HUD 渲染  │                                   │
│  └──────────────────┘                                   │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

### 9.3 各进程职责

| 进程 | 语言 | 职责 | 对外端口 |
|------|------|------|----------|
| `mavlink_router.py` | Python | 串口 ↔ UDP MAVLink 双向转发 | UDP 14550 |
| `web_dashboard.py` | Python | HTTP + WebSocket, 推送遥测 JSON | TCP 8080, 8081 |
| `video_streamer` | C++ | 摄像头采集 + MPP 编码 + UDP 推流 | UDP 5600 |
| `test_mavlink_control.py` | Python | 控制指令交互 + HDMI HUD 渲染 | 无 |

---

## 10. 数据流汇总

```
数据流 1: PX4 遥测 → 所有显示端
═══════════════════════════════
PX4 ──UART3──→ [mavlink_router] ──UDP:14550──→ QGC
                    │
                    ├──→ 解析 JSON ──WebSocket:8081──→ 网页仪表盘
                    │
                    └──→ 更新缓存 ──→ HDMI HUD (本地)


数据流 2: 控制指令 → PX4
═══════════════════════════════
[test_mavlink_control] ──→ UART3 ──→ PX4     (本地直接控制)
[QGC] ──UDP:14550──→ [mavlink_router] ──→ UART3 ──→ PX4  (远程控制)


数据流 3: 摄像头 → 网页/QGC
═══════════════════════════════
[Camera] ──USB──→ [video_streamer]
                    │
                    ├──→ MPP H.264 → UDP:5600 ──→ QGC
                    │
                    └──→ (阶段1) MJPEG → HTTP:8080/video ──→ 网页


数据流 4: MAVLink 指令透传
═══════════════════════════════
QGC ──UDP:14550──→ [mavlink_router] ──→ UART3 ──→ PX4
(地面站 LAND/RTL/模式切换 → 直接透传到飞控)
```

---

## 11. 开发实施计划

### 阶段顺序

| 阶段 | 内容 | 依赖 | 工作量 |
|------|------|------|--------|
| **P0** | HDMI 本地 HUD (纯文本遥测+指令) | 现有 test_mavlink_control.py | 小 |
| **P1** | MJPEG 视频推流 (摄像头 → 浏览器) | 摄像头驱动正常 | 小 |
| **P2** | 网页仪表盘 (遥测 JSON + 控制状态 + MJPEG 视频) | P0 + P1 | 中 |
| **P3** | MAVLink UDP 转发 (Zero3W → QGC) | 无 | 小 |
| **P4** | H.264 硬编码替代 MJPEG | P1 | 中 |
| **P5** | 网页 MSE 播放 H.264 | P4 | 中 |
| **P6** | 视觉追踪结果叠加到画面 | 视觉模块开发完成 | 小 |

### 可并行开发

```
P0 (HDMI HUD) ──────────────────────┐
P1 (MJPEG视频) ─────────────────────┤
P3 (MAVLink转发) ───────────────────┼──→ P2 (网页仪表盘)
                                     │
P4 (H.264) ──→ P5 (MSE) ────────────┘
```

### 最小可行产品 (MVP)

只做 P0+P1+P4+P3 就可以：
- HDMI 屏幕显示遥测
- QGC 连上 UDP 看到全部数据
- QGC 里能看到视频

**不需要网页仪表盘**。QGC 本身就是最好的地面站。

---

## 附录 A: 关键端口一览

| 端口 | 协议 | 方向 | 用途 |
|------|------|------|------|
| 22 | TCP/SSH | 电脑 → Zero3W | 调试/部署 |
| 14550 | UDP | 双向 | MAVLink 遥测 + 指令 |
| 5600 | UDP | Zero3W → 电脑/QGC | H.264 视频流 |
| 8080 | TCP/HTTP | Zero3W → 浏览器 | 网页仪表盘 |
| 8081 | TCP/WebSocket | Zero3W → 浏览器 | 遥测实时推送 |

## 附录 B: 目录文件清单

```
zero3w_test/zero_3w_mavlink/
├── test_mavlink_serial.py          # (已有) 串口通信测试
├── test_mavlink_control.py         # (已有) 飞行控制测试
├── DISPLAY_AND_WIFI_PLAN.md        # 本文档
│
├── mavlink_router.py               # (新增) MAVLink 串口↔UDP 转发
├── web_dashboard.py                # (新增) HTTP + WebSocket 仪表盘
├── video_streamer.cpp              # (新增) MPP H.264 编码 + UDP 推流
│
├── static/                         # (新增) 网页静态文件
│   ├── index.html
│   ├── dashboard.js
│   └── style.css
│
└── hud_renderer.py                 # (新增) HDMI HUD 渲染模块
```
