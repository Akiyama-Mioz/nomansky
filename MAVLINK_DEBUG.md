# MAVLink 通信调试日志

> 从零实现 Zero3W ↔ PX4 MAVLink v1 通信的全过程记录。

---

## 一、背景

### 目标

用 C++ 替代 Python pymavlink，在 Zero3W 上实现对 PX4 飞控的 MAVLink v1 串口通信。

### 硬件

- Zero3W UART3 (`/dev/ttyS3`, Pin3=RX, Pin5=TX) ↔ PX4 TELEM1
- 波特率：115200 8N1
- 协议：MAVLink v1 (STX=0xFE)

---

## 二、错误 1：手写 MAVLink 解析器 CRC 校验失败

### 现象

程序运行后遥测全部为零（`mode=` 空字符串，`armed=NO`，`alt=0.0`，`bat=0.0V`）。

### 排查过程

1. Python pymavlink 测试 — **正常**，PX4 在发送 HEARTBEAT/ATTITUDE/...
2. 原始串口数据测试 — 串口有数据流入 (8608 字节/3秒)
3. C 解析器 `parse_errors=32` (30 秒内 32 个 CRC 错误)，**0 条成功消息**

### 根因

手写的 `mavlink_minimal.h` 中，CRC 计算有一个错误：

**错误代码**：
```c
// CRC 校验: 仅对 rx_buf 中的帧头计算 CRC
uint8_t crc_extra = _mav_crc_extra(p->msgid & 0xFF);
uint16_t crc_exp = mav_crc(&p->rx_buf[1], 9 + p->payload_len, crc_extra);
```

问题：payload 字节存储在 `p->payload[]`，而 CRC 计算时只用 `p->rx_buf[]`（仅含帧头+STX，不含 payload）。CRC 输入与实际帧内容不匹配，必然失败。

**修复**：
```c
// 分两段计算 CRC：帧头(不含STX, 从 rx_buf) + payload(从 p->payload)
uint16_t crc_exp = 0xFFFF;
for (int i = 1; i <= 9; i++)
    crc_exp = _mav_crc_accumulate(p->rx_buf[i], crc_exp);
for (int i = 0; i < p->payload_len; i++)
    crc_exp = _mav_crc_accumulate(p->payload[i], crc_exp);
crc_exp = _mav_crc_accumulate(crc_extra, crc_exp);
```

### 修复后结果

`parse_errors=0` — CRC 校验不再报错，但**仍然 0 条成功消息**。

---

## 三、错误 2：MAVLink v2 解析器无法处理 v1 帧

### 现象

修复 CRC bug 后，100% 的字节被丢弃，0 条消息通过。

### 排查过程

1. 用 `sudo cat /dev/ttyS3 | od -A x -t x1z` 查看原始字节 — **0 字节输出** (阻塞模式无数据)
2. 用 Python 主动发送 MAVLink v2 HEARTBEAT 并读取响应：

```python
ser.write(v2_heartbeat_frame)  # 发送 0xFD 开头的 v2 帧
rx = ser.read(1024)
# rx[0] = 0xFE — PX4 回复的是 v1 帧!
```

### 根因

手写解析器只处理 `MAVLINK_STX = 0xFD`（v2 协议），而 PX4 发送的是 `0xFE`（v1 协议）。

**v1 帧格式**：
```
字节 0:    0xFE (STX)
字节 1:    payload 长度
字节 2:    序列号
字节 3:    系统 ID
字节 4:    组件 ID
字节 5:    消息 ID (1 字节)
字节 6+:  payload
最后 2:   CRC-16/MCRF4XX
```

**v2 帧格式** (我们解析器支持的)：
```
字节 0:    0xFD (STX)
字节 1:    payload 长度
字节 2-3:  兼容标志 (v1 没有)
字节 4:    序列号
字节 5:    系统 ID
字节 6:    组件 ID
字节 7-9:  消息 ID (3 字节)
字节 10+: payload
最后 2:   CRC-16/MCRF4XX
```

v1 和 v2 的帧头长度不同 (5 vs 9 字节)，消息 ID 长度不同 (1 vs 3 字节)，无法用同一个解析器处理。

**修复**：重写解析器支持双协议自动检测：
```c
case MAV_STATE_STX:
    if (byte == 0xFD) {
        p->mav_version = 2;  // v2
    } else if (byte == 0xFE) {
        p->mav_version = 1;  // v1
    }
```

v1 路径跳过 v2 的 incompat/compat flags 和 2 个额外的 msgid 字节。

### 修复后结果

仍然 0 条消息，`parse_errors=21` (30 秒) — CRC 匹配上了约定，但值不对。

---

## 四、错误 3：C 语言 designated initializer 不兼容 C++11

### 现象

编译错误：
```
sorry, unimplemented: non-trivial designated initializers not supported
```

### 根因

`mavlink_minimal.h` 中的 CRC_EXTRA 表使用了 C99 designated initializer：
```c
static const uint8_t _mav_crc_extra_map[256] = {
    [0]   = 50,   // HEARTBEAT
    [30]  = 39,   // ATTITUDE
    ...
};
```

GCC 在 `-std=c++11` 模式下不支持此语法。

### 修复

改为 switch-case 函数：
```c
static inline uint8_t _mav_crc_extra(uint32_t msgid) {
    switch (msgid) {
        case 0:   return 50;
        case 30:  return 39;
        ...
    }
}
```

---

## 五、改用官方 MAVLink C 库

### 决策

发现手写 MAVLink 解析器调试成本过高，且 `mavlink_minimal.h` 的 CRC_EXTRA 值无法验证。改用 pymavlink 生成的官方 C MAVLink 库。

### 生成

```bash
python3 /usr/local/bin/mavgen.py \
  --lang=C --wire-protocol=2.0 \
  --output=mavlink/ \
  common.xml
```

生成 143 个标准 C 头文件。初次用 `--wire-protocol=1.0` 生成，但后续发现 v1.0 库的 parser 内部有问题，改为 v2.0（向后兼容 v1 输入）。

### 使用方式

- **解析**：`mavlink_frame_char_buffer()` — 调用者提供 buffer 版本，无全局状态
- **编码**：`mavlink_msg_xxx_pack()` + `mavlink_msg_to_send_buffer()`
- **解码**：`mavlink_msg_xxx_decode()`
- **CRC**：`crc_accumulate_buffer()` + `crc_accumulate()`

### 验证

用 pymavlink 抓取的真实 HEARTBEAT 帧（17 字节），直接喂给 C 库的 `mavlink_frame_char_buffer()`：
```
SUCCESS: base=0x1d custom=0x3040000 status=0
```
与 pymavlink 解析结果完全一致。**官方 C 库完全正确**。

---

## 六、错误 4：串口配置不是 raw 模式 — 根因

### 现象

改用官方 C 库后，`mavlink_frame_char_buffer` 离线测试完美通过，但实际串口运行时仍然 `msgs=0, parse_errors=18`。

### 排查过程

**关键对比测试** — Python 和 C 从同一串口读原始字节：
```python
# Python pyserial
ser = serial.Serial('/dev/ttyS3', 115200)
raw = ser.read(1024)
# 结果: 1024 bytes, 27 个 v1 STX (0xFE)
# 首字节: fe 1c 81 01 01 ...
```

```c
// C 代码 (手工 memset termios)
fd = open("/dev/ttyS3", ...);
// c_cflag = CS8 | CLOCAL | CREAD; c_iflag = IGNPAR;
// 结果: 158 bytes, 1 个 v1 STX
// 首字节: 12 d5 cb 01 ...
```

**Python 和 C 从同一串口读到了完全不同的字节流！**

### 根因

C 代码的串口初始化：
```c
struct termios t;
memset(&t, 0, sizeof(t));     // ← 全部清零
t.c_cflag = CS8 | CLOCAL | CREAD;
t.c_iflag = IGNPAR;
t.c_cc[VTIME] = 0;
t.c_cc[VMIN] = 0;
```

`memset` 清零遗漏了关键操作——`cfmakeraw()` 负责清除的很多标志位。特别是：

| 标志位 | memset 后 | cfmakeraw 后 | 影响 |
|--------|:---------:|:------------:|------|
| IGNBRK | 0 (未清除?)* | 清除 | 串口 break 信号处理 |
| BRKINT | 0 | 清除 | break 信号 → SIGINT |
| PARMRK | 0 | 清除 | 奇偶校验错误标记 |
| ISTRIP | 0 | 清除 | 7-bit 字符截断 |
| INLCR | 0 | 清除 | NL→CR 转换 |
| IGNCR | 0 | 清除 | 忽略 CR |
| ICRNL | 0 | 清除 | CR→NL 转换 |
| IXON | 0 | 清除 | XON/XOFF 流控 |
| OPOST | 0 | 清除 | 输出处理 |
| VMIN | 0 | **1** | 最少读取字节数 |

> *注：memset 清零后这些标志位确实是 0（已清除），但 `cfmakeraw` 还做了额外的设置，包括设置 VMIN=1。**VMIN=0 → VMIN=1 是最关键的差异**。

VMIN=0, VTIME=0 的配置让 `read()` 在任何情况下都立即返回（0 字节或少量字节），导致字节流不连续，MAVLink 帧被截断。

### 修复

完全匹配 pyserial 的 raw 模式配置：
```c
struct termios t;
tcgetattr(fd, &t);           // 获取现有设置
cfmakeraw(&t);                // raw 模式基础
t.c_cflag |= (CLOCAL | CREAD);
t.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
t.c_cflag |= CS8;
t.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
t.c_oflag &= ~OPOST;
t.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
t.c_cc[VMIN]  = 1;           // ← 关键! 等待至少 1 字节
t.c_cc[VTIME] = 0;
```

### 修复后结果

```
[MAVLINK] first message received (msgid=32)
[MAVLINK] reader loop stopped (msgs=696, parse_errors=12)

[0] 0 balloons | mode=AUTO armed=NO alt=0.0 bat=0.0V
[30] 1 balloons | mode=AUTO armed=NO alt=0.0 bat=65.5V
```

**首次正确收到 MAVLink 遥测数据！**

---

## 七、最终架构

### MAVLink 通信栈

```
Zero3W 应用层
  ├── MavlinkReader (后台线程)
  │     ├── select() + read(serial_fd)      // 非阻塞读
  │     ├── mavlink_frame_char_buffer()      // 官方 C 库逐字节解析
  │     └── mavlink_msg_xxx_decode()         // 官方 C 库消息解码
  ├── PX4Controller (主线程)
  │     ├── mavlink_msg_xxx_pack()           // 官方 C 库消息编码
  │     ├── mavlink_msg_to_send_buffer()     // 官方 C 库序列化
  │     └── write(serial_fd, buf)            // 串口发送
  └── TelemetryData (共享缓存)
        ├── std::atomic 字段 (roll/pitch/yaw/vx/vy/vz...)  // 无锁
        └── std::mutex 字段 (mode_name/state_name)          // 字符串
```

### 只用官方库做编解码

**不使用** `mavlink_parse_char()` — 此函数依赖全局 channel buffer (`m_mavlink_buffer[]`)，在多线程环境中不够清晰。

**使用** `mavlink_frame_char_buffer()` — 调用者提供 `mavlink_message_t` 和 `mavlink_status_t` buffer，无全局状态。

---

## 八、时间线

| 阶段 | 耗时 | 问题 | 解决 |
|------|:---:|------|------|
| 1 | ~30min | 手写 CRC bug (payload 未纳入 CRC 计算) | 分两段 CRC |
| 2 | ~20min | v1/v2 协议不匹配 | 双协议自动检测 |
| 3 | ~10min | C11 designated init | switch-case 函数 |
| 4 | ~30min | 改用官方 MAVLink C 库 | pymavlink mavgen 生成 |
| 5 | ~60min | 官方库 `mavlink_parse_char` channel buffer 不工作 | 换 `mavlink_frame_char_buffer` |
| 6 | ~40min | 离线测试通过，实时运行失败 | **发现串口 cfg 问题** |
| 7 | **~90min** | **串口 raw 模式缺失 (VMIN=0 vs 1)** | **cfmakeraw + VMIN=1** |

**总耗时约 4.5 小时**，最终问题不在 MAVLink 协议层，而在 Linux 串口驱动的 termios 配置。

---

## 九、经验教训

1. **先验证物理层**。串口不通时，不要直接怀疑协议栈。先用 Python pymavlink 确认 PX4 在发送正确数据。
2. **不要自己造轮子**。手写 MAVLink 解析器有太多细节（CRC_EXTRA 值、v1/v2 协议差异）。直接生成标准 C MAVLink 库并以此为基础。
3. **用 cfmakeraw 配置串口**。手工 memset termios 会遗漏关键标志位。pyserial 作为参考实现是最好的文档。
4. **离线测试 + 在线测试结合**。用已知正确的帧数据进行离线验证，确认编解码正确后，再去排查物理层问题。这样能快速定位问题层级。
5. **VMIN=1 是关键**。在非阻塞模式下，VMIN=0 导致 `read()` 返回不连续的字节流，MAVLink 帧同步完全丢失。
