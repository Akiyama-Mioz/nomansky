# Balloon Tracker — C++ 架构文档

> 无人机机载气球自主追踪系统，运行于 Radxa Zero 3W (RK3566, ARM aarch64)。

---

## 一、系统概览

```
┌──────────────────────────────────────────────────────────────────┐
│                       Radxa Zero 3W                              │
│                                                                  │
│  IMX219 ─→ GStreamer ─→ NPU(YOLO) ─┐                            │
│                                     ├─→ HUD ─→ /dev/fb0 (HDMI)  │
│  PX4 ─→ UART3 ─→ MAVLink Thread ───┘                            │
│                  (串口唯一读者)                                    │
│                                                                  │
│  控制指令: Keyboard ─→ PX4Controller ─→ UART3 ─→ PX4             │
└──────────────────────────────────────────────────────────────────┘
```

### 硬件连接

| Zero3W 引脚 | 连接目标 |
|------------|---------|
| CSI (MIPI) | IMX219 摄像头 |
| UART3 (Pin 3 RX, Pin 5 TX) | PX4 TELEM1 |
| HDMI | 显示器 |
| USB | 键盘 (可选) |

---

## 二、线程架构

**双线程模型**：

| 线程 | 职责 | 频率 |
|------|------|------|
| **主线程** | GStreamer 采集 → NPU 推理 → HUD 渲染 → fb 输出 → 键盘交互 | ~13.4 fps |
| **MAVLink 后台线程** | 串口 `select` + `read` → `mavlink_frame_char_buffer` 解析 → 更新 `TelemetryData` 共享缓存 | 事件驱动 (~77 msg/s) |
| **OFFBOARD 心跳线程** (按需) | 维持 OFFBOARD 模式的 20Hz 零速指令 | 仅在 OFFBOARD 模式 |

```
主线程 (渲染循环)                    MAVLink 线程 (串口唯一读者)
┌─────────────────────┐              ┌──────────────────────────┐
│ gst_pull_frame()    │              │ select(100ms timeout)    │
│ gst_pull_selfpath() │              │   ↓ 有数据               │
│ rknn_infer()        │              │ read()                  │
│ draw_hud()          │  TelemetryData│   ↓                     │
│ fb_write()          │◄────共享 ────│ mavlink_frame_char_buf()│
│ cv::waitKey(1)      │  (atomic)     │   ↓ 完整帧              │
└─────────────────────┘              │ dispatch(msg)           │
        13.4 fps                     │   → _telemetry (atomic) │
                                     └──────────────────────────┘
                                             事件驱动
```

**为什么不是三线程（如 Python 版）？**

1. C++ 没有阻塞的 `input()` 交互菜单，主循环 = 渲染循环。
2. RKNN 推理是同步阻塞的，开独立线程不会更快，反而增加帧传递和同步开销。
3. 串口必须独立线程，因为 `read()` 会阻塞，不能让它卡住渲染循环。

---

## 三、数据流

```
                   ┌─── 数据源 ───┐
                   │              │
              ┌────┴────┐   ┌────┴─────┐
              │ IMX219  │   │ PX4 飞控  │
              │ 摄像头   │   │ (TELEM1)  │
              └────┬────┘   └────┬─────┘
                   │             │ UART3, 115200 8N1, MAVLink v1
                   │             │
    ┌──────────────▼──────────┐  │
    │ GStreamer 双管道         │  │
    │ /dev/video0 → 1080p NV12│  │
    │ /dev/video1 → 640p NV12 │  │
    └──────────────┬──────────┘  │
                   │             │
             cv::cvtColor        │
             NV12 → BGR          │
                   │             │
    ┌──────────────▼──────────┐  │
    │ rknn_infer(detect_frame)│  │
    │ YOLO 320×320 → boxes[]  │  │
    └──────────────┬──────────┘  │
                   │             │
           缩放 boxes 到 1080p   │
                   │             │
                   │    ┌────────▼──────────┐
                   │    │ MavlinkReader     │
                   │    │ 后台线程           │
                   │    │ serial → parse    │
                   │    │ → TelemetryData   │
                   │    └────────┬──────────┘
                   │             │
                   │    TelemetrySnapshot
                   │    (值拷贝, 无锁读)
                   │             │
            ┌──────▼──────┬──────▼──────┐
            │   hud.render()            │
            │   Layer 1: 相机背景       │
            │   Layer 2: 检测框(绿色)   │
            │   Layer 3: HUD 面板(82%)  │
            └─────────────┬────────────┘
                          │
                   cv::cvtColor BGR→BGRA
                   lseek + write /dev/fb0
                          │
                   ┌──────▼──────┐
                   │  HDMI 显示器 │
                   └─────────────┘
```

**数据流特点**：所有传感器数据在主线程的 `hud.render()` 处汇合，单向流向 HDMI 屏幕。无回流。

---

## 四、控制流

```
┌─ 下行控制 (主线程 → PX4) ──────────────────────────────────────┐
│                                                                 │
│  键盘输入 / 自主逻辑                                             │
│       │                                                        │
│       ▼                                                        │
│  PX4Controller::set_mode("OFFBOARD")                          │
│  PX4Controller::arm()                                         │
│  PX4Controller::takeoff(3.0)                                  │
│  PX4Controller::send_offboard_velocity(vx, vy, vz)            │
│       │                                                        │
│       ▼                                                        │
│  mavlink_msg_xxx_pack() → mavlink_msg_to_send_buffer()        │
│       │                                                        │
│       ▼                                                        │
│  write(serial_fd, buf) → PX4                                   │
│       │                                                        │
│  等待反馈 (轮询 shared_telemetry + ACK 队列):                   │
│    - ACK 到达 → 命令成功/失败                                   │
│    - 模式变化 → 切换成功                                        │
│    - 高度到达 → 起飞完成                                        │
│    - 超时 → 失败                                               │
└─────────────────────────────────────────────────────────────────┘

┌─ 上行数据 (PX4 → 主线程) ──────────────────────────────────────┐
│                                                                 │
│  PX4 → 串口 → MavlinkReader 线程 → TelemetryData               │
│       → 两个消费者:                                              │
│          1. HUD 渲染 (显示遥测)                                  │
│          2. 控制逻辑 (确认命令成功)                               │
└─────────────────────────────────────────────────────────────────┘
```

**命令-确认闭环**：
```
发命令 ────────────────────────→ PX4 执行
                                  │
                                  ▼
轮询 TelemetryData ◄──── MAVLink 上行消息
  │
  ├─ ACK 到达? → 成功/失败/拒绝
  ├─ 模式变了? → 切模式成功
  ├─ 高度够了? → 起飞完成
  └─ 超时? → 失败
```

---

## 五、模块结构

```
balloon_tracker_cpp/
├── main.cc                 496行  主入口 + 主循环 (GStreamer/NPU/HUD/键盘)
├── mavlink_reader.h/cc     370行  串口读取 + MAVLink 解析线程
├── px4_controller.h/cc     500行  PX4 飞控指令 (模式/解锁/起飞/速度控制)
├── hud_renderer.h/cc       416行  HUD 渲染 + HDMI 输出
├── shared_data.h           113行  跨线程共享数据结构 (TelemetryData/CmdState/Box)
├── mavlink_minimal.h       669行  轻量 MAVLink v2 实现 (已被官方库替代，保留备用)
├── mavlink/                143个  官方 MAVLink C 头文件 (pymavlink mavgen 生成)
└── Makefile                 45行  Zero3W 板载编译
```

### 各模块职责

| 模块 | 有无内部线程 | 核心 API |
|------|:---:|---------|
| **MavlinkReader** | ✅ 后台线程 | `open()`, `snapshot()`, `drain_ack()` |
| **PX4Controller** | ✅ 心跳线程(按需) | `set_mode()`, `arm()`, `takeoff()`, `send_offboard_velocity()` |
| **HUDRenderer** | ❌ 纯函数 | `render(frame, boxes, tele, cmd)` |
| **GStreamer** (main.cc 内) | ❌ 同步 pull | `gst_pull_frame()`, `gst_pull_selfpath()` |
| **RKNN** (main.cc 内) | ❌ 同步调用 | `rknn_infer(cv::Mat)` |

---

## 六、MAVLink 通信架构

```
Zero3W (sysid=255, compid=190, 伪装为 GCS)
      ↕ MAVLink v1, 115200 8N1, UART3
PX4   (sysid=1, compid=1, TELEM1)

解析: mavlink_frame_char_buffer() — 官方 C 库
      调用者提供 buffer, 无全局 channel state
编码: mavlink_msg_xxx_pack() — 官方 C 库
CRC:  crc_accumulate_buffer() — 官方 C 库
```

**共享遥测结构**：
- 高频数值字段 (roll, pitch, yaw, vx, vy, vz 等) 用 `std::atomic<float>` — 无锁读写
- 字符串字段 (mode_name, state_name) 用 `std::mutex` 保护
- `snapshot()` 返回 `TelemetrySnapshot` (纯值类型)，一次拷贝，线程安全

---

## 七、HUD 渲染架构

### 三层叠加

```
┌─────────────────────────────────────────────┐
│ Layer 1: 相机画面 (1920×1080 BGR)            │
│   或纯黑背景 (相机未就绪时)                   │
├─────────────────────────────────────────────┤
│ Layer 2: 检测框                               │
│   - 绿色矩形 + "balloon XX%" 标签            │
│   - 坐标已在 1080p 坐标系                    │
├─────────────────────────────────────────────┤
│ Layer 3: HUD 面板 (左上角, 256px 宽)          │
│   - 82% 不透明度 (cv::addWeighted)           │
│   - TELEMETRY: Link/Mode/Armed/State/       │
│     Roll/Pitch/Yaw/Alt/Vel/Bat/EKF          │
│   - COMMANDS: Last Cmd/Result/Vel Cmd/      │
│     最近 3 条历史                             │
│   - LOG: 最近 5 条                            │
│   - 帧计数 + 运行时间                         │
└─────────────────────────────────────────────┘
           │
    cv::cvtColor BGR → BGRA
    逐行 lseek + write → /dev/fb0
```

### HDMI 输出设计

使用 `lseek + write` 逐行写入 framebuffer，而非单次 8MB write 或 mmap 直接赋值。
原因：RK3566 AXI 总线上，numpy mmap 的 8MB 单次 DMA burst 与 UART DMA 竞争，导致进程 D 状态死锁。参见 `docs/development/DEBUG_LOG.md`。

---

## 八、性能数据 (60 秒稳态运行)

| 指标 | 数值 | 说明 |
|------|------|------|
| 帧率 | 13.4 fps | GStreamer + NPU + HUD |
| CPU 占用 | ~103% (1 核) | 82.9% user + 20.3% sys |
| NPU 占用 | 12% | YOLO 320×320, 0.8 TOPS NPU |
| 内存 (RSS) | ~3.5 MB | 非常精简 |
| MAVLink 速率 | ~77 msg/s | 4736 条/60s |
| 二进制大小 | 134 KB | stripped, 动态链接 |

### CPU 热点

| 系统调用 | 调用次数/60s | 占比 | 说明 |
|----------|:---------:|:----:|------|
| write | 261,369 | 40% | HDMI framebuffer 逐行输出 |
| lseek | 260,284 | 26% | 每行 fb 写入前定位 |
| futex | 6,243 | 29% | 遥测线程 mutex 竞争 |
| read | 159 | <1% | 串口 (select 模式) |

---

## 九、未实现功能

| 功能 | 当前状态 | 说明 |
|------|---------|------|
| **追踪状态机** | ❌ 未实现 | SEARCH→LOCK→TRACK→HOVER→LOST 五阶段 (见 `docs/design/VISION_DESIGN.md`) |
| **IBVS 视觉伺服** | ❌ 未实现 | 用检测框面积/位置导引无人机运动 |
| **颜色追踪** | ❌ 未实现 | 锁定后高频 (30Hz+) H-S 直方图 back-projection 追踪 |
| **Kalman 滤波** | ❌ 未实现 | 目标丢失后的预测搜索窗 |
| **自主飞行** | ❌ 未实现 | 目前仅键盘手动控制，无自动任务 |
| **WiFi 视频串流** | ❌ 未实现 | MPP H.264 编码 + WebSocket/HTTP 推流 (见 `docs/design/DISPLAY_AND_WIFI_PLAN.md`) |
| **MAVLink UDP 转发** | ❌ 未实现 | Zero3W 作为 PX4↔QGC 的 MAVLink 路由器 |
| **DRM 显示** | ❌ 未实现 | 替换 /dev/fb0, 更好的垂直同步和 GPU 加速 |
| **IMX219 ISP 参数调节** | ❌ 未实现 | 曝光/增益自适应控制 |

---

## 十、优化方向

### P0 — 高收益、低风险

| 优化项 | 当前开销 | 预期收益 |
|--------|---------|---------|
| **FB 全屏单次 write** | 1080 行 × ~13fps = ~14,000 次 lseek+write/秒 | 节省 ~26% 内核时间 |
| **减少 cv::Mat clone** | 每帧 clone 一次 1080p 画面用于画框 | 节省 ~8MB 内存分配/帧 |

### P1 — 中等收益

| 优化项 | 说明 |
|--------|------|
| **GStreamer 零拷贝** | 避免 NV12→BGR 的 CPU 拷贝，用 GPU shader 或 ISP 直出 BGR |
| **无锁遥测队列** | 替换 `std::mutex` + `std::queue` 为 lock-free ring buffer，减少 futex 开销 |
| **GStreamer blocking pull** | `gst_app_sink_try_pull_sample` → 用信号量阻塞等待，消除 100ms busy-poll |

### P2 — 长期

| 优化项 | 说明 |
|--------|------|
| **DRM/KMS 显示** | 替换 /dev/fb0，更好的垂直同步，减少撕裂 |
| **多线程渲染** | 采集/推理/渲染流水线化，提高帧率 |
| **GPU 加速 resize** | 用 Mali-G52 做 BGR→BGRA 转换和缩放 |
| **RKNN 模型优化** | int8 量化模型已在使用，可尝试模型剪枝或更小的 backbone |

---

## 十一、设计原则

1. **串口只有一个读者** — 避免多线程竞争 DMA。MAVLink 读取线程是串口的唯一所有者。
2. **主线程即渲染线程** — 不引入独立的渲染线程，HUD 刷新率 = 相机帧率。
3. **共享遥测用 atomic + 值拷贝** — `TelemetryData` 内部 atomic 字段无锁读写，`snapshot()` 返回可拷贝的纯值类型 `TelemetrySnapshot`。
4. **官方库做编解码** — MAVLink 消息的编码/解码/CRC 全用官方 C 库，自写部分只有字节级 v1 帧解析和 dispatch。
5. **不做过度抽象** — GStreamer pipeline 和 RKNN 推理保持 inline 实现，不创建不必要的类包装。
