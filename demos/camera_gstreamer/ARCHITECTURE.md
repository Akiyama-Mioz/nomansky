# main.cc 架构文档

> 文件: `zero3w_yolo_camera_gstreamer_demo/main.cc`  
> 功能: GStreamer 双管线摄像头采集 + YOLO 气球检测 + HDMI framebuffer 显示

---

## 一、整体管线

```
IMX219 MIPI 传感器 (3280×2464 @ 21fps, 计划切 1920×1080 @ 30fps)
    │
    ▼ (硬线)
rkisp ISP
    │
    ├── mainpath → /dev/video0 (NV12 1920×1080 @ 21fps)
    │       │
    │       ▼ (GStreamer v4l2src)
    │   ┌──────────────────────────────┐
    │   │  gst_pull_frame()             │
    │   │  NV12 → BGR (cv::cvtColor)   │  ← 显示帧, ~19ms
    │   │  → fb_show() → write fb0     │
    │   └──────────────────────────────┘
    │
    └── selfpath → /dev/video1 (NV12 640×480 @ 21fps)
            │
            ▼ (GStreamer v4l2src)
        ┌──────────────────────────────┐
        │  gst_pull_selfpath()          │
        │  NV12 → BGR (~1ms)            │  ← YOLO 用, 快 20 倍
        │  → rknn_infer() → boxes[]     │
        └──────────────┬───────────────┘
                       │
                       ▼ (坐标缩放 640×480 → 1920×1080)
        ┌──────────────────────────────┐
        │  draw_boxes() → fb_show()    │
        │  HDMI 屏幕                    │
        └──────────────────────────────┘
```

**双管线分工：mainpath 管显示画质，selfpath 管 AI 速度。** ISP 硬件天生支持双路输出，不冲突。

---

## 二、模块划分 (~280 行)

```
main.cc
├── 常量                        MODEL_W/H, CONF/NMS_THR, DISP_W/H
├── 全局                        g_running, g_ctx, g_nbox, fb_fd
├── fb_init/show/close          framebuffer 打开/全屏 write/关闭
├── gst_pull_frame()            mainpath GStreamer pipeline → BGR
├── gst_pull_selfpath()         selfpath GStreamer pipeline → BGR
├── rknn_load/unload            NPU 模型加载/释放
├── rknn_infer()                YOLOv8 推理 + 解码 + NMS
├── draw_boxes()                cv::rectangle + cv::putText
└── main()                      主循环: pull → detect → draw → show
```

---

## 三、主循环逻辑

```cpp
while (g_running) {
    // 1. Mainpath 帧 (1920×1080, 显示用)
    cv::Mat frame = gst_pull_frame();    // try_pull_sample(100ms)

    // 2. Selfpath 帧 (640×480, YOLO 用, ~1ms cvtColor)
    cv::Mat detect = gst_pull_selfpath(); // try_pull_sample(50ms)
    bool has_self = !detect.empty();

    // 3. YOLO 推理 (优先 selfpath, 退 mainpath)
    auto boxes = has_self ? rknn_infer(detect) : rknn_infer(frame);

    // 4. 坐标缩放 + 画框 + 写屏
    float sx = 1920.0 / detect.cols, sy = 1080.0 / detect.rows;
    for (auto &b : boxes) { b *= {sx, sy}; }
    draw_boxes(frame, boxes);
    fb_show(frame);  // BGR → BGRA → write /dev/fb0
}
```

---

## 四、GStreamer 双 Pipeline

| | mainpath | selfpath |
|------|---------|---------|
| 设备 | `/dev/video0` | `/dev/video1` |
| 分辨率 | 1920×1080 | 640×480 |
| 用途 | 显示 | YOLO 推理 |
| NV12→BGR 耗时 | ~19ms | **~1ms** |
| pull 超时 | 100ms | 50ms |

---

## 五、DRM 双平面探索（未采用）

### 5.1 目标

```
Plane 72 (Esmart): 摄像头 NV12 → 硬件 YUV→RGB (零 CPU)
Plane 56 (Smart):  HUD BGRA 透明画布 (CPU 渲染, 硬件叠加)
```

### 5.2 尝试过的方案

| 方案 | 做法 | 结果 |
|------|------|------|
| 纯 DRM atomic | `drmModeAtomicCommit` 同时配两个 plane | ❌ Rockchip VOP 只暴露一个 plane object (id=96), Smart/Esmart 是内部窗口 |
| 半 DRM: kmssink + fb0 | kmssink 写 Plane 72, fb0 写 Plane 56 | ❌ kmssink `force-modesetting` 霸占全部 plane; 不加则摄像头不显示 |
| GPU EGL+GBM | `glupload→glcolorconvert→gldownload` | ✅ gst-launch 可行, ❌ appsink 收不到帧 |

### 5.3 根因

Rockchip VOP 驱动将 Smart(56)、Esmart(72)、Cluster(96) 三个硬件窗口封装在同一个 DRM plane object 内，`drmModeSetPlane` 够不着内部窗口。摄像头通过 `kmssink plane-id=72` 私有属性访问 Esmart，但这个路径与 fb0 的 fbcon 驱动冲突——无法同时拥有两个 DRM master。

### 5.4 结论

当前保持 **单 plane + fb0** 方案。等后续自己写 EGL+GBM shader 做 NV12→RGB GPU 转换，或者走 GStreamer `compositor` 做画面合成。

---

## 六、性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| 帧率 | ~12.5-14 fps | CPU 色彩转换限制; 传感器 21fps 锁 |
| YOLO 频率 | 每帧 (12.5 Hz) | NPU 远未到瓶颈 (12%) |
| CPU 利用率 | 136% (1.36/4 核) | 双 GStreamer 管道 + 色彩转换 |
| NPU 利用率 | 12% | 320×320 模型太轻 |
| 端到端延迟 | ~75ms (平均) | 曝光→ISP→采集→推理→画框→显示 |
| selfpath cvtColor | ~1ms | 640×480, 比 1080p 快 20 倍 |
| mainpath cvtColor | ~19ms | 1920×1080, 当前瓶颈 |

### strace 分布 (30s)

| 系统调用 | 占比 | 说明 |
|----------|------|------|
| futex | 33% | GStreamer 线程同步 |
| wait4 | 25% | 子进程等待 |
| ppoll | 20% | 帧等待 |
| sched_yield | 14% | CPU 空转 (~21 万次/30s) |
| write | 3% | fb0 写入 |

### 延迟拆解

```
传感器曝光         ~5ms
ISP 管线            ~2ms
selfpath 采集+BGR   ~4ms
YOLO 预处理+推理    ~10ms
画框+FB write      ~15ms
显示扫描            ~16ms (60Hz)
传感器帧间隔        ≤47ms (21fps)
─────────────────────────
平均               ~75ms
最坏               ~99ms
```

---

## 七、剩余 CPU 余量预估

当前占 1.36 核 (34%)。后续新增：

| 模块 | 预估 | 说明 |
|------|------|------|
| MAVLink Rx/Tx | 0.1 核 | 串口解析 + 20Hz 发指令 |
| 控制状态机 + IBVS | 0.15 核 | 像素误差→速度指令 |
| HUD 遥测渲染 | 0.1 核 | cv::putText 少量文字 |
| WiFi UDP 转发 | 0.03 核 | sendto/recvfrom |
| **合计** | **~1.8 核 (45%)** | 剩余 55% |

---

## 八、文件清单

| 文件 | 用途 |
|------|------|
| `main.cc` | 主程序 |
| `Makefile` | 板端编译 |
| `drm_display.h/cc` | DRM 双平面尝试 (未采用, 保留参考) |
| `ARCHITECTURE.md` | 本文档 |
| `CHANGELOG.md` | 变更日志 |

---

## 九、未来方向

1. **传感器切 30fps**：改 IMX219 overlay 触发 2×2 binning, 帧率天花板从 21→30
2. **DRM 显示**：自写 EGL+GBM shader 做 GPU NV12→RGB, 或 GStreamer compositor 合成
3. **MAVLink C++ 集成**：串口通信搬进独立线程, 共享变量与主循环通信
4. **多线程拆分**：采集/检测/显示/控制 独立线程, 互不阻塞
