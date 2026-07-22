# 无人机机载计算机串口通信 & HDMI HUD — 开发调试实录

> **硬件**: Radxa Zero 3W (RK3566) + PX4 飞控  
> **连接**: UART3 串口 (115200 8N1) + HDMI 显示器  
> **语言**: Python 3 + pymavlink + OpenCV  
> **时间**: 2026 年 7 月  
> **目标**: 为秋招面试准备一份完整的嵌入式系统调试案例

---

## 目录

1. [项目背景](#1-项目背景)
2. [阶段一: 串口基础通信验证](#2-阶段一-串口基础通信验证)
3. [阶段二: 飞行控制指令测试](#3-阶段二-飞行控制指令测试)
4. [阶段三: HDMI HUD 显示器开发](#4-阶段三-hdmi-hud-显示器开发)
5. [阶段四: DMA 冲突排查与解决](#5-阶段四-dma-冲突排查与解决)
6. [最终架构](#6-最终架构)
7. [面试要点总结](#7-面试要点总结)

---

## 1. 项目背景

### 1.1 整个大项目

使用机载计算机（Zero3W）通过摄像头识别气球、控制无人机自主接近气球。Zero3W 与 PX4 飞控之间通过 MAVLink 协议通信。

### 1.2 本次子任务

1. 验证 Zero3W ↔ PX4 串口通信正常
2. 实现基础飞行控制指令（模式切换、解锁、起飞等）
3. 开发 HDMI HUD 叠加显示，实时展示飞控遥测数据

### 1.3 硬件拓扑

```
Zero3W (RK3566)                  PX4 飞控
┌──────────────┐           ┌──────────────┐
│ UART3        │──TX/RX──→│ TELEM1       │
│ Pin3=RX      │  115200  │ MAVLink v2   │
│ Pin5=TX      │  8N1     │              │
│              │           │              │
│ HDMI         │──→ 屏幕   │              │
└──────────────┘           └──────────────┘
```

---

## 2. 阶段一: 串口基础通信验证

### 2.1 初始状态

- PX4 飞控已配置 MAV_1_CONFIG=101 (TELEM 1), MAV_1_MODE=2 (Onboard)
- Zero3W UART3 已通过设备树 overlay 启用 (`rk3568-uart3-m0.dtbo`)
- 串口设备: `/dev/ttyS3`, 波特率 115200

### 2.2 遇到的问题：串口不存在

Zero3W 刷机后 UART3 默认是关闭的。`/dev/ttyS3` 根本不存在。

**排查过程**：
```bash
ls /dev/ttyS*        # 只看到 /dev/ttyS1, 没有 ttyS3
dmesg | grep serial  # UART1 有注册, UART3 没有
ls /boot/dtbo/       # UART3 overlay 是 .disabled 状态
```

**解决方案**：
1. 启用设备树 overlay: `mv rk3568-uart3-m0.dtbo.disabled rk3568-uart3-m0.dtbo`
2. 修改 `/boot/extlinux/extlinux.conf`，在 `fdtoverlays` 行中添加 `rk3568-uart3-m0.dtbo`
3. 重启 → `/dev/ttyS3` 出现

**学到的**：嵌入式 Linux 的外设不是开箱即用的，需要理解设备树 (Device Tree) 机制。引脚功能复用需要通过 dtbo overlay 配置。

### 2.3 串口权限问题

`/dev/ttyS3` 属于 `root:dialout`，radxa 用户无权访问。

```bash
# 错误: Permission denied
python3 test_mavlink_serial.py

# 解决: 加入 dialout 组 或 用 sudo
sudo usermod -aG dialout radxa
```

### 2.4 基础通信测试脚本

`test_mavlink_serial.py`: 使用 pymavlink 库连接串口，等待 PX4 心跳包（HEARTBEAT），验证双向通信。同时发送 STATUSTEXT 和 PING 测试双向通道。

**测试结果**：
```
✅ 收到 PX4 心跳包
✅ PING 往返 RTT=6.4ms
✅ 持续接收 20 种消息类型
```

**学到的**：MAVLink 协议的心跳机制 (1Hz HEARTBEAT) 是判断通信正常的最快方式。PING/RTT 可以直接量化通信延迟。

---

## 3. 阶段二: 飞行控制指令测试

### 3.1 问题：PX4 拒绝接受指令

串口通信正常，但发送模式切换 (SET_MODE)、解锁 (ARM) 等指令后，PX4 不执行。

**排查**：
```python
# 查看飞控状态
HEARTBEAT: system_status=0 (UNINIT)
```

**PX4 状态机**: UNINIT → BOOT → CALIBRATING → STANDBY → (才能 ARM)

**根因**: PX4 卡在 UNINIT 状态。因为：
- 没有 GPS 模块 → 等 GPS 锁定
- 没有遥控器接收机 → 等 RC 信号
- 仅 USB 供电 → 电池监测读数为 65535mV (无效值)

**解决方案**：通过 QGC 设置 PX4 参数跳过这些检查：
```
COM_ARM_WO_GPS  = 1       # 无 GPS 也允许初始化
COM_RC_IN_MODE   = 1      # 无遥控器也允许
CBRK_SUPPLY_CHK = 894281  # 跳过供电检查
```

### 3.2 问题：模式显示错误

脚本发送 SET_MODE 后，PX4 返回的 custom_mode 解析不正确，总是显示 "MANUAL"。

**排查**：
```
发送前 custom_mode: 0x06040000
发送后 custom_mode: 0x04040000  ← 变了！
COMMAND_ACK: result=0 (ACCEPTED) ← PX4 接受了！
```

**根因**: PX4 的 `custom_mode` 编码依赖固件版本。旧代码用 `custom_mode & 0xFF` 取主模式，但实际主模式在 bits 16-23：
```python
# 错误
main = custom_mode & 0xFF          # → 0, 永远是 "MANUAL"

# 正确
main = (custom_mode >> 16) & 0xFF  # → 4, "AUTO"
```

**学到的**：MAVLink 协议中不同飞控固件对 custom_mode 的编码不同。不能假设位偏移。COMMAND_ACK 比 HEARTBEAT 更可靠地反映指令是否被接受。

### 3.3 飞行控制脚本

`test_mavlink_control.py`: 封装了 PX4Controller 类，支持：
- 模式切换 (GUIDED, OFFBOARD, RTL...)
- Arm/Disarm
- Takeoff/Land
- GUIDED 位置控制
- OFFBOARD 速度控制（带缓启动/缓停）

支持交互式菜单和自动测试序列两种模式。

---

## 4. 阶段三: HDMI HUD 显示器开发

### 4.1 需求

在 HDMI 屏幕上实时显示飞控遥测数据，类似 MSI Afterburner 的紧凑叠加层。方便开发调试时不需要看终端。

### 4.2 技术方案

**Linux Framebuffer (`/dev/fb0`)**：屏幕就是一块显存，往里面写像素数据就能显示。

```
帧缓冲 = 1920×1080×4 bytes (BGRA) = 8MB
通过 mmap 映射到用户空间 → 直接当数组读写
```

**渲染方案**：OpenCV (cv2) 在 numpy 数组上画文字和图形 → 转换像素格式 → 写入 `/dev/fb0`

### 4.3 架构设计：模块分离

```
hud_renderer.py  ──── 纯渲染模块 (HUD 类)
    ├── open()         打开 framebuffer, 自动检测分辨率/像素格式
    ├── update(tele, cmd)  接收数据 → 渲染 → 写入 fb
    └── close()        清理资源

test_mavlink_control.py ──── 控制模块
    └── --hud 参数     导入 HUD, 后台线程渲染, 不与串口冲突
```

**关键设计决策**：HUD 模块不碰串口。它只接收从外部传入的 telemetry 字典。这样两个模块可以独立开发、测试、复用。

### 4.4 Framebuffer 参数自动检测

不同设备、不同 HDMI 分辨率、不同色深下，framebuffer 格式不同。不能写死 1920×1080 BGRA。

```python
# 通过 ioctl FBIOGET_VSCREENINFO 读取实际参数
fcntl.ioctl(fd, FBIOGET_VSCREENINFO, buf)
# 解析: xres, yres, bits_per_pixel, red/green/blue offset+length

# 自动判断像素格式:
if bpp == 32 and R@16 and B@0: → "BGRA"
if bpp == 16 and R@11 and B@0: → "RGB565"
```

**学到的**：写嵌入式显示程序必须处理多种像素格式。直接写死参数 = 换个屏幕就炸。

### 4.5 中文显示为 `?`

**现象**: 画面上的中文全部显示为 `?`。

**根因**: OpenCV 内置的 Hershey 字体只支持 ASCII，不支持中文。

**解决**: 全部标签改为英文。同时将布局从全屏改为紧凑的左上角竖排列表。

---

## 5. 阶段四: DMA 冲突排查与解决

**这是整个开发过程中最有价值的调试经历。**

### 5.1 症状

HUD 启动后流畅运行约 10 秒（100 帧），然后**画面冻结**，帧计数器停止增长。

**非常稳定的复现**：每次都在 ±20 帧内卡住。

### 5.2 逐步排查

#### 假说 1: 串口数据流断了？

```python
# 诊断: 只读串口, 不写屏, 用 console 打印
while True:
    msg = mav.recv_match(blocking=False)
    print(msg.get_type())
# 结果: ✅ 连续运行 20s+, 数据流完全正常
```

**排除**。

#### 假说 2: Framebuffer 写入有问题？

```python
# 诊断: 只写屏 (递增数字), 不读串口
for i in range(300):
    draw_frame(i)
    time.sleep(1)
# 结果: ✅ 30 帧全部正常显示
```

**但注意**：fbtest 是 1 秒 1 帧。HUD 是 0.1 秒 1 帧（10Hz）。

**排除**。

#### 假说 3: 内存分配太多？

每帧 `cv2.cvtColor` 创建新的 8MB 数组 → 累积 GC 压力？

改成预分配缓冲区：
```python
bgra_buf = np.zeros((h, w, 4), dtype=np.uint8)  # 只分配一次
cv2.cvtColor(canvas, cv2.COLOR_BGR2BGRA, dst=bgra_buf)  # 复用
```
**没用，还是卡**。排除。

#### 假说 4: pymavlink 内部消息队列满了？

改用最底层的 `parse_buffer()` 逐字节喂解析器，不经过 `recv_match` 的缓冲。

**没用**。排除。

#### 假说 5: 刷新频率太高？

从 10Hz 降到 5Hz → 卡在 ~100 帧。从每秒帧数看：
- 10Hz × 10s = 100 帧
- 5Hz × 20s = 100 帧

**总计帧数一样！** 这说明不是帧数问题，是**累积 DMA 操作量**问题。

#### 关键实验: 单线程 vs 双线程

```python
# 实验 A: 主循环完全不写 fb (只串口+打印)
# 结果: ✅ 20s 无限运行

# 实验 B: 主循环完全不读串口 (只写 fb+假数据)
# 结果: ✅ 60s 无限运行, 334 帧完美

# 实验 C: 两者同时跑
# 结果: ❌ ~100 帧必卡
```

**结论**: 串口和 framebuffer **各自独立工作时都正常**。同在一个线程交替执行时，~100 帧后冻结。

#### 关键证据: 进程状态

```bash
ps aux | grep hud_renderer
# root  662  Dl  ...  python3 hud_renderer.py
#              ↑
#         D = 不可中断睡眠 (卡在内核 I/O)
```

进程状态 `D`（uninterruptible sleep）——这个进程卡在 Linux 内核里，等待某个 I/O 操作完成，而这个 I/O 永远不会完成。这是 **DMA 死锁** 的典型表现。

### 5.3 根因分析

```
RK3566 内部总线架构:

  CPU ──→ AXI 总线 ──→ 内存控制器 ──→ DDR 内存
                ↑            ↑
                │            │
           UART3 DMA    HDMI 显示控制器 (framebuffer DMA)
           (~12KB/s)    (每次刷新 8MB, 10Hz = 80MB/s)
```

**串口 DMA**（接收 UART 数据写入内存）和 **framebuffer DMA**（从内存读取像素送 HDMI）共享 RK3566 的 AXI 总线。当两个 DMA 同时高速运转时：

1. numpy `fb_arr[:] = bgra` 触发一次 8MB 的 DMA 突发传输
2. UART 中断同时到达，需要 DMA 读取新数据
3. 内存控制器仲裁失败，DMA 描述符耗尽
4. 其中一个 DMA 永远等不到完成 → 进程进入 D 状态 → 画面冻结

### 5.4 解决方案对比

| 方案 | 结果 |
|------|------|
| numpy mmap 写 (`fb_arr[:] = bgra`) | ❌ ~100 帧卡 |
| `os.write()` 系统调用写 | ✅ **完美解决** |
| 拆成双线程 | 略有改善 (80→126 帧) 但不彻底 |

**`os.write()` 为什么有效**：

`os.write(fd, raw)` 走 VFS（虚拟文件系统）→ 内核 `fb_write` 函数。这个路径上内核会：
1. 使用独立的 DMA 通道（不与 UART 共享中断线）
2. 必要时自动分块传输（单次不超过某个阈值）
3. 正确管理 DMA 描述符的分配和回收

而 numpy 的直接内存赋值 `fb_arr[:] = bgra` 是一次 8MB 的 memcpy，直接走 CPU 的 `memcpy` → 触发硬件预取 → AXI 突发传输。一次传 8MB 不释放总线，UART 的数据进不来。

### 5.5 经验总结

1. **DMA 冲突是嵌入式系统开发中容易被忽视的问题**。单功能测试通过 ≠ 集成后正常。
2. **进程状态 `ps aux` 是排错利器**。`D` 状态直接指向内核 I/O 阻塞。
3. **对比实验设计很重要**：A 正常 + B 正常 ≠ A+B 正常。交叉组合测试才能定位。
4. **numpy 的便利性有代价**：`ndarray[:] = other` 看起来简洁，但底层是一次巨大的 memcpy，没有内核 I/O 层的保护。

---

## 6. 最终架构

```
hud_renderer.py (独立模块)
├── HUD 类
│   ├── open()           ioctl 检测分辨率/pixel format
│   ├── update(tele,cmd) 遥测→渲染→os.write(fb)
│   └── _flip()          预分配缓冲, os.write VFS 路径
│
└── standalone main()
    ├── 线程 A: 串口 read + parse_buffer 解析 MAVLink → 更新 tele
    └── 线程 B: 每 100ms 从 tele 渲染 → os.write /dev/fb0

test_mavlink_control.py (控制模块)
├── PX4Controller 类
│   ├── connect/close
│   ├── set_mode / arm / disarm / takeoff / land / rtl
│   ├── offboard_velocity_ramp (带缓启动)
│   └── read_telemetry / print_status
│
└── --hud 参数: import HUD, 后台线程调用 hud.update()
      共享 _shared_telemetry (mutex), 单串口不冲突
```

**数据流**:
```
PX4 ──串口──→ [线程A: read+parse] ──→ tele dict
                                         │
                                    [线程B: render] ──os.write()──→ /dev/fb0 → HDMI
```

---

## 7. 面试要点总结

### 7.1 可以讲的亮点

| 亮点 | 关键词 |
|------|--------|
| 自底向上排查 DMA 冲突 | 假设-实验-排除、系统调用 vs 裸内存操作 |
| 设备树 overlay 启用外设 | Device Tree, pin mux, dtbo |
| MAVLink 协议解析 | HEARTBEAT, COMMAND_ACK, custom_mode 编码 |
| Linux framebuffer 编程 | ioctl, mmap, pixel format 自适应 |
| 模块化架构 | 依赖注入、职责分离、可复用设计 |
| 生产者-消费者模型 | 线程拆分、共享状态 |

### 7.2 如果面试官问"你遇到最大的技术困难是什么？"

> 在 RK3566 上同时使用串口 DMA 和 HDMI framebuffer DMA 时，两者在 AXI 总线上冲突导致进程进入 D 状态死锁。通过设计对照实验（A 正常、B 正常、A+B 异常），逐步排除串口、fb、内存分配、频率等因素，最终定位到 numpy 的裸内存赋值触发 8MB 突发 DMA 与 UART DMA 的仲裁失败。解决方案是用 `os.write()` 系统调用替代 numpy 直接赋值——系统调用路径上的内核代码会正确处理 DMA 分块和中断。

### 7.3 涉及的技术栈

- **通信协议**: MAVLink v2, UART/串口 (115200 8N1)
- **系统编程**: Linux framebuffer, ioctl, mmap, sysfs, 设备树
- **语言/库**: Python 3, pymavlink, OpenCV (cv2), numpy
- **硬件**: RK3566 (ARM Cortex-A55), PX4 飞控, Radxa Zero 3W
- **调试方法**: 假设-验证-排除、进程状态分析、对照实验
