# 重构变更日志

> 从 `zero3w_yolo_camera_demo/main.cc` (V4L2) → `zero3w_yolo_camera_gstreamer_demo/main.cc` (GStreamer 双管线)

---

## 一、重构动机

原版 `main.cc` 使用原生 V4L2 Multiplanar API 采集摄像头帧，存在两个核心问题：

1. **S_FMT 破坏 rkaiq 校准**：`cam_open()` 调 `ioctl(S_FMT)` 改 ISP 格式 → rkaiq 3A 丢失 → 画面偏绿
2. **Multiplanar API 不兼容 OpenCV/Python**：Python 无法用 OpenCV 打开 `/dev/video0`

改用 GStreamer `v4l2src`：格式协商由 GStreamer 处理，不暴力覆写 ISP 配置。

---

## 二、改动对比

| 方面 | V4L2 原版 | GStreamer 初版 | GStreamer 当前版 |
|------|----------|-------------|-----------------|
| 采集方式 | 原生 V4L2 ioctl | gst_parse_launch (单管) | **双管线** (mainpath + selfpath) |
| 显示帧 | 1080p NV12→BGR (19ms) | 1080p NV12→BGR (19ms) | 1080p NV12→BGR (19ms) |
| YOLO 帧 | 1080p→320 (一同) | 1080p→320 (19ms cvt) | **640×480→320 (1ms)** |
| 代码量 | ~300 行 | ~220 行 | ~280 行 |
| ISP 配置 | S_FMT → 发绿 | GStreamer 不改 → 正常 | GStreamer 不改 → 正常 |
| FB 写入 | 1080 次 write | 1 次 write | 1 次 write |
| 帧率 | ~21 fps | ~14 fps | ~12.5 fps |
| NPU 利用率 | 1 Hz | 14 Hz (每帧) | 12% (每帧, 320×320) |
| 二进制 | 3MB 静态 | 68KB 动态 | 68KB 动态 |
| 编译 | 交叉编译 | 板端编译 | 板端编译 |

---

## 三、好在哪里

1. **颜色正常**：GStreamer 不调 `S_FMT`，ISP 保持 camera-hdmi 预设格式，rkaiq 有效
2. **selfpath 双管线**：YOLO 用 640×480 小图, NV12→BGR 只要 ~1ms (1080p 要 19ms)
3. **ISP 硬件双路**：mainpath + selfpath 是 Rockchip ISP 原生设计，互不阻塞
4. **代码干净**：每个 pipeline 封装成独立函数, 后续换采集源只改一处
5. **板端编译**：g++ 直编, 68KB 秒传
6. **每帧推理**：NPU 从 1Hz → 14Hz，延迟 ~75ms < 100ms
7. **线程安全**：主循环单线程, selfpath 帧无帧时自动 fallback 到 mainpath

---

## 四、差在哪里

1. **帧率下降**：21fps → 12.5fps，双 GStreamer 管道线程调度开销 (~1.36 核)
2. **CPU 空转**：`sched_yield` ~21 万次/30s (14% CPU)，可改成阻塞式 `pull_sample`
3. **GPU 未用上**：EGL+GBM 无窗上下文创建成功, 但 `appsink + GPU` 有兼容问题
4. **DRM 双平面未跑通**：Rockchip VOP 驱动限制 (详见 ARCHITECTURE.md §五)
5. **传感器锁 21fps**：IMX219 驱动跑在 3280×2464 模式, 未切到 1920×1080 2x2 binning 30fps

---

## 五、遇到的问题与解决

| # | 问题 | 根因 | 解决 |
|---|------|------|------|
| 1 | 画面发绿 | V4L2 `S_FMT` 重配 ISP → rkaiq 校准丢失 | camera-hdmi 预设 ISP 格式, GStreamer 不调 `S_FMT` |
| 2 | GStreamer 拉不到帧 | Caps 中 `framerate=30/1` 与摄像头 21fps 不匹配 | 去掉帧率限制, GStreamer 自协商 |
| 3 | GPU `appsink` 收不到帧 | `glupload→glcolorconvert→gldownload` 输出未正确传递 | 降回 CPU 管线 |
| 4 | ISP 队列卡死 | 多次 GPU 测试后异常退出未释放 `/dev/video0` | 重启恢复 |
| 5 | `cv::Mat` wrap 失败 | `gldownload` 输出 BGR stride 有 padding | 用 `map.size/height` 算实际 stride |
| 6 | `sched_yield` 过多 | `try_pull_sample` 返回 false 后立即重试, 无 sleep | 加 `usleep(10000)` |
| 7 | Python 端无法采集 | RK ISP Multiplanar V4L2, OpenCV 不支持 | 全 C++ 路线, Python 做 MAVLink 参考 |
| 8 | `balloon-ai.service` 自启干扰 | systemd 服务未停, 杀进程后重拉 | `systemctl disable` + `pkill` |
| 9 | 纯 DRM 双平面失败 | VOP driver 只暴露一个 plane object (id=96) | 保持 fb0 方案, DRM 代码保留参考 |
| 10 | 半 DRM (kmssink+fb0) 失败 | kmssink `force-modesetting` 霸占全部 plane; 不加则摄像头不显示 | fb0 方案 |
| 11 | 多推理压 NPU 导致卡顿 | 7 次推理/帧 = 56ms 仅推理, 帧时间 90ms+ → 帧率崩 | 还原 1 次/帧, NPU 12% 是正常的 |
| 12 | 摄像头 overlay 未加载 | extlinux.conf 缺少 `radxa-zero3-rpi-camera-v2.dtbo` | 添加到 fdtoverlays, 重启 |
| 13 | IMX219 驱动未触发 | UVC 驱动加载了但没检测到摄像头 (USB vs MIPI) | 加载 IMX219 overlay + rkaiq 初始化 |
| 14 | MIPI vs USB 摄像头混用 | 原以为摄像头走 UVC, 实际是 MIPI CSI + ISP | 改用 GStreamer v4l2src 走 ISP |

---

## 六、性能演进

| 阶段 | 帧数 | 推理 | YOLO 频率 | 帧率 | NPU | 管线 |
|------|------|------|----------|------|-----|------|
| V4L2 原版 | 577/28s | 28 | 1 Hz | ~21 fps | 未知 | 单路 1080p |
| GStreamer 1.0s | 383/28s | 27 | 1 Hz | ~14 fps | 3% | 单路 1080p |
| GStreamer 0.5s | 383/28s | 55 | 2 Hz | ~14 fps | 6% | 单路 1080p |
| GStreamer 0.2s | 383/28s | 128 | 4.6 Hz | ~14 fps | 8% | 单路 1080p |
| GStreamer 每帧 | 383/28s | 383 | 14 Hz | ~14 fps | 12% | 单路 1080p |
| **selfpath 每帧** | **188/15s** | **188** | **12.5 Hz** | **~12.5 fps** | **12%** | **双路 1080p+640p** |
| selfpath 60s | 771/60s | 771 | 12.9 Hz | ~12.9 fps | 12% | 双路 1080p+640p |

---

## 七、DRM 双平面探索记录

### 尝试 1：纯 DRM atomic

用 `drmModeAtomicCommit` 同时驱动 Plane 56 (HUD) 和 Plane 72 (摄像头)。

**结果**：`drmModeGetPlaneResources` 只返回 1 个 plane (id=96)。Rockchip VOP 把 Smart/Esmart/Cluster 封装在一个 plane object 内, 无法独立操作。

### 尝试 2：半 DRM (kmssink + fb0)

- `kmssink plane-id=72` 放摄像头 → Plane 72
- `write(/dev/fb0)` 写 HUD → Plane 56

**结果**：`force-modesetting=true` 时 kmssink 霸占全部 plane, fb0 被覆盖。`force-modesetting=false` 时摄像头不显示。两个 DRM master 无法共存。

### 尝试 3：GPU EGL+GBM

设置 `GST_GL_PLATFORM=egl GST_GL_GBM_DRM_DEVICE=/dev/dri/card0`, 用 `glupload→glcolorconvert→gldownload` 做 GPU 色彩转换。

**结果**：`gst-launch-1.0` 验证可行 (GBM GL 上下文创建成功)，但 C++ `appsink` 收不到 GPU 管线的帧 (GStreamer 已知问题)。

### 当前状态

保持单 plane + fb0。DRM 代码保留在 `drm_display.{h,cc}` 作为参考。

---

## 八、文件清单

| 文件 | 用途 |
|------|------|
| `main.cc` | 主程序 (双管线 GStreamer + YOLO + fb0 显示) |
| `drm_display.h` | DRM 双平面接口 (未采用) |
| `drm_display.cc` | DRM 双平面实现 (未采用) |
| `main_drm.cc` | DRM 版本入口 (未采用) |
| `Makefile` | 板端编译 |
| `ARCHITECTURE.md` | 架构文档 |
| `CHANGELOG.md` | 本文档 |

---

## 九、待办事项

| 优先级 | 项目 | 方向 |
|------|------|------|
| **P0** | 传感器切 30fps | IMX219 overlay 改 2×2 binning, 天花板 21→30fps |
| P1 | CPU 空转 | `try_pull_sample` → 阻塞 `pull_sample` |
| P1 | 帧率恢复 20fps+ | DRM plane 硬件显示 / GPU 色彩转换 |
| P2 | MAVLink C++ 集成 | 独立线程串口通信 |
| P2 | HUD 遥测渲染 | cv::putText 叠加 MAVLink 数据 |
| P3 | 多线程架构 | 采集/检测/显示/控制 独立线程 |
| P3 | 模型升级 | 重训 640×640 YOLO, 提升小目标精度 |
| P3 | WiFi MAVLink 转发 | UDP 转发 Zero3W→QGC |
