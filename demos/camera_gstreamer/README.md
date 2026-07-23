# Camera + YOLO GStreamer Demo

> 摄像头气球检测测试项目 — 验证 MIPI 摄像头采集、NPU 推理、HDMI 显示的端到端管线。

**这是 `balloon_tracker` 项目的测试/验证代码，不属于最终产品。** 最终产品代码在 `src/` 中。

## 功能

- GStreamer 双管线采集 (mainpath 1080p 显示 + selfpath 640p AI)
- RKNN NPU YOLOv8 气球检测 (每帧推理, ~12.5fps)
- HDMI framebuffer 显示 (检测框 + 置信度标注)

## 编译 (在 Zero3W 板端)

```bash
cd demos/camera_gstreamer
make
```

## 运行

```bash
# 先初始化 ISP
sudo systemctl start camera-hdmi && sleep 5 && sudo systemctl stop camera-hdmi

# 跑检测
sudo LD_LIBRARY_PATH=/usr/local/lib ./zero3w_camera_ai ballon_int8.rknn
```

## 依赖

- `libgstreamer1.0-dev` `libgstreamer-plugins-base1.0-dev`
- `libopencv-dev` `libdrm-dev`
- `rknn-toolkit-lite2` (librknnrt.so)

## 文档

- [ARCHITECTURE.md](ARCHITECTURE.md) — 管线架构、性能指标、DRM 探索
- [CHANGELOG.md](CHANGELOG.md) — 完整变更记录
