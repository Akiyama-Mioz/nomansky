# AI Agent 开发指南

> 读完本文档即可开始在这个项目上写代码。预计阅读时间: 10 分钟。

---

## 一、项目是什么

无人机**气球自主追踪系统**的机载计算机部分。

```
Radxa Zero 3W (RK3566, ARM aarch64)  ← 你现在在这台机器上开发
  ├── 串口 UART3 (/dev/ttyS3, 115200 8N1) ↔ PX4 飞控 (MAVLink v2)
  ├── USB 摄像头 (后续接)
  └── HDMI 屏幕 (显示实时遥测)
```

**当前阶段**: 串口通信已打通，HUD 屏幕显示已稳定，正在完善控制+显示集成。

---

## 二、快速开始

### 连接 Zero3W

```bash
# 网络
IP:   192.168.66.223
用户: radxa
密码: radxa
SSH:  ssh radxa@192.168.66.223

# 需要 root 权限的地方用:
echo radxa | sudo -S <command>
```

### 部署代码

```bash
# 从开发机部署到 Zero3W
sshpass -p 'radxa' scp -o StrictHostKeyChecking=no <file> radxa@192.168.66.223:/home/radxa/

# 杀掉旧进程再运行
ssh radxa@192.168.66.223
echo radxa | sudo -S pkill -9 python3
sudo python3 /home/radxa/hud_renderer.py
```

### 运行测试

```bash
# 1. 基础串口测试 (安全, 只读不发)
sudo python3 test_mavlink_serial.py

# 2. 控制测试 (会发指令! 不装桨)
sudo python3 test_mavlink_control.py --mode guided

# 3. HUD 显示 (在 HDMI 屏上看遥测)
sudo python3 hud_renderer.py
```

---

## 三、核心文件

| 文件 | 行数 | 做什么 | 何时用 |
|------|------|--------|--------|
| `test_mavlink_serial.py` | 800+ | 验证串口物理连接 | 首次接线后 |
| `test_mavlink_control.py` | ~900 | 飞行控制脚本 | 测试飞控指令 |
| `hud_renderer.py` | 436 | HDMI 屏幕 HUD 显示 | 实时看遥测 |
| `HUD_PARAMS.md` | 260 | HUD 每个参数的含义 | 查参数定义 |
| `DEBUG_LOG.md` | 400 | DMA 冲突排查全记录 | 面试准备 |
| `DISPLAY_AND_WIFI_PLAN.md` | 653 | WiFi 视频推流规划 | 后续开发 |
| `DEV_STATUS.md` | 300 | 代码逻辑 + 问题清单 | 了解当前状态 |

### 文件关系图

```
hud_renderer.py                   test_mavlink_control.py
┌─────────────────┐              ┌──────────────────────┐
│ HUD 类 (纯渲染)  │◄──import────│ PX4Controller (控制)  │
│                 │  (--hud)    │                      │
│ HUD.update()    │              │ read_telemetry()     │
│ HUD._log()      │              │ arm/takeoff/land ... │
│                 │              │                      │
│ main() 独立运行  │              │ main()               │
│ 自己连串口+显示  │              │ 交互菜单+自动测试     │
└─────────────────┘              └──────────────────────┘
      ✅ 稳定                        ✅ 控制可用
                                     ❌ --hud 集成有 bug
```

---

## 四、当前已知问题

### Issue #1: --hud 集成模式姿态显示为 0.0

**优先级**: P0（不影响独立版使用，但后续需要修）

**症状**: `test_mavlink_control.py --hud` 时 Roll/Pitch/Yaw 始终为 0.0

**根因**: 
1. `read_telemetry()` 原来只存 `roll` (弧度), HUD 读 `roll_deg` (度) → 字段不匹配 (已部分修复)
2. `_shared_telemetry` 只在 `read_telemetry()` 被调用时更新，交互式菜单空闲时不更新

**修复方向**: 让控制脚本空闲时持续后台调用 `read_telemetry()`，或在 HUD 线程里用独立 fd 读串口

**绕过方案**: 暂时用独立的 `hud_renderer.py` 看遥测，用 `test_mavlink_control.py` 控制飞机

### Issue #2: PX4 卡在 UNINIT 状态

**原因**: 没有 GPS、遥控器、电池（USB 供电），PX4 初始化检查过不了

**解决**: 通过 QGC 设置跳过参数：`COM_ARM_WO_GPS=1`, `COM_RC_IN_MODE=1`, `CBRK_SUPPLY_CHK=894281`

---

## 五、开发注意事项

### 串口不能多进程共用

`/dev/ttyS3` 同时只能被一个进程打开。不要让两个脚本同时连串口。

### HUD 渲染的关键教训

| 做过的事 | 结果 |
|----------|------|
| numpy `fb_arr[:] = bgra` 直接写 mmap | ❌ ~100 帧后画面冻结 |
| `os.lseek+os.write` 逐行写 | ✅ 稳定 |
| 每行调用 `.tobytes()` | ❌ 画面出现小方框 |
| 全面板一次 `tobytes()` 后逐行切片 | ✅ 稳定 |
| BGR→BGRA 转换 (`cv2.cvtColor`) | ⚠️ 占 34% CPU, 可优化 |

### 中文显示问题

OpenCV 内置 Hershey 字体不支持中文。所有屏幕文字必须用 ASCII。

### PX4 custom_mode 解析

不同 PX4 固件版本的 custom_mode 编码不同。当前固件:
```python
main_mode = (custom_mode >> 16) & 0xFF   # 主模式
sub_mode  = (custom_mode >> 24) & 0xFF   # 子模式
```

---

## 六、常用调试命令

```bash
# 在 Zero3W 上
ls /dev/ttyS*                          # 查看可用串口
dmesg | grep tty                       # 串口内核日志
cat /proc/interrupts | grep tty        # 串口中断计数
ps aux | grep python3                  # 查看进程状态 (D=卡死)
sudo pkill -9 python3                  # 强杀所有 Python 进程

# 性能分析
strace -f -c -p PID                    # 系统调用统计
perf stat -p PID -- sleep 10           # CPU 性能计数
perf record -p PID -g -- sleep 8 && perf report  # 热点函数

# 部署
sshpass -p 'radxa' scp file.py radxa@192.168.66.223:/home/radxa/
```

---

## 七、MAVLink 消息速查

| 消息 | 频率 | 用途 |
|------|------|------|
| HEARTBEAT #0 | 1 Hz | 心跳, 模式, 解锁状态 |
| ATTITUDE #30 | 50-100 Hz | roll/pitch/yaw |
| LOCAL_POSITION_NED #32 | 20-50 Hz | 位置, 速度 |
| GLOBAL_POSITION_INT #33 | 10-20 Hz | GPS, 高度 |
| BATTERY_STATUS #147 | 0.5 Hz | 电池电压 |
| ESTIMATOR_STATUS #230 | 2-5 Hz | EKF 健康状态 |
| STATUSTEXT #253 | 事件 | 日志/错误消息 |
| COMMAND_ACK #77 | 事件 | 命令确认 |

---

## 八、后续开发路径

```
现在 ──→ 修 --hud 集成 ──→ 实机飞行测试 ──→ 接入视觉追踪
                              │
                              └── 先装桨 + GPS, 在 GUIDED 下起飞
                                  验证 arm/takeoff/land 正常
```

### 如果要新加功能

1. **加一个 HUD 显示字段**: 改 `hud_renderer.py` 的 `update()` 里的 `add()` 调用
2. **加一个控制指令**: 改 `test_mavlink_control.py` 的 `PX4Controller` 类
3. **加一个 MAVLink 消息解析**: 改 `hud_renderer.py` 的 `serial_loop()` 或 `test_mavlink_control.py` 的 `read_telemetry()`

### 参考文档

- [HUD_PARAMS.md](HUD_PARAMS.md) — 每个显示参数的含义和数据来源
- [DEV_STATUS.md](DEV_STATUS.md) — 代码结构详解和完整问题清单
- [DEBUG_LOG.md](DEBUG_LOG.md) — DMA 问题排查的完整方法论
