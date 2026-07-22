# 气球视觉识别方案 — 设计参考文档

> **架构核心**: YOLO 低频检测 + 高频颜色追踪 (双模互补)  
> **控制模式**: 基于图像的视觉伺服 (IBVS)，面积比替代绝对距离  
> **依赖**: RKNN NPU (YOLO推理) + OpenCV (颜色追踪/轮廓分析)  
> **目标平台**: Radxa Zero 3W (RK3568, aarch64)  
> **最后更新**: 2026-07-21

---

## 目录

1. [方案总览](#1-方案总览)
2. [五大阶段概览](#2-五大阶段概览)
3. [阶段一: 目标发现 SEARCH](#3-阶段一-目标发现-search)
4. [阶段二: 目标锁定 LOCK](#4-阶段二-目标锁定-lock)
5. [阶段三: 高频追踪 TRACK](#5-阶段三-高频追踪-track)
6. [阶段四: 目标保持 HOVER](#6-阶段四-目标保持-hover)
7. [阶段五: 异常恢复 LOST](#7-阶段五-异常恢复-lost)
8. [关键数据结构](#8-关键数据结构)
9. [OpenCV 核心操作详解](#9-opencv-核心操作详解)
10. [颜色模型: H-S 直方图 vs 固定阈值](#10-颜色模型-h-s-直方图-vs-固定阈值)
11. [面积比控制原理](#11-面积比控制原理)
12. [卡尔曼预测器](#12-卡尔曼预测器)
13. [多阶段控制指令映射](#13-多阶段控制指令映射)
14. [参数配置表](#14-参数配置表)
15. [完整流程图](#15-完整流程图)
16. [错误处理与边缘情况](#16-错误处理与边缘情况)
17. [性能分析](#17-性能分析)
18. [文件清单与接口](#18-文件清单与接口)
19. [开发优先级](#19-开发优先级)
20. [调试与可视化](#20-调试与可视化)

---

## 1. 方案总览

### 1.1 为什么不用单目距离估计

单目相机天然缺乏深度信息。传统方案需要：
- 已知相机内参 (fx, fy, cx, cy) — 需要标定
- 已知目标物理尺寸 — 气球直径非标准
- 平面地球假设 — 不适用于空中悬浮气球

### 1.2 核心思想: 面积比消去尺度

小孔成像模型下，物体在图像上的面积与距离的平方成反比：

```
A / A₀ = (d₀ / d)²

其中: A   = 当前帧气球最小外接圆面积
      A₀  = 参考帧气球最小外接圆面积
      d   = 当前距离
      d₀  = 参考距离 (未知, 但在比例中消去)
```

**目标不是飞到某个绝对距离 (N 米)，而是飞到参考距离的一个比例 (如 0.5 倍)**:

```
scale_factor = √(A₀ / A)    范围: (0, ∞)
    1.0  = 参考距离
    0.5  = 参考距离的一半 (接近了)
    2.0  = 参考距离的两倍 (更远了)
```

**优点**: 不需要相机标定、不需要知道气球直径、不需要地面假设。

### 1.3 为什么用最小外接圆而非原始轮廓面积

```
水滴形气球:
   ┌─────────┐
   │    ●    │  ← minEnclosingCircle (半径由最宽处决定)
   │  ◠◠◠   │
   │ ◠   ◠  │  ← 原始轮廓 (被拉长、变形)
   │ ◠   ◠  │
   │  ◠◠◠   │
   └─────────┘

原始轮廓面积: 随气球姿态 (被绳子拉成水滴形) 波动 ±30%
外接圆面积:   只由最宽处决定 → 波动 < ±5%
```

### 1.4 双模互补设计

| | YOLO 模式 | 颜色追踪模式 |
|---|---|---|
| **频率** | 0.5 Hz (每 2 秒一次) | 25-30 Hz (每帧) |
| **计算资源** | NPU (专用硬件) | CPU (OpenCV) |
| **适用距离** | 任意 | 近距 (bbox > 40×40 px) |
| **光照适应性** | 强 (特征级) | 中 (自适应采样) |
| **精度** | 粗 (矩形 bbox) | 精 (像素级轮廓) |
| **失败模式** | 误检/漏检 | 同色干扰/光照剧变 |
| **角色** | 发现 + 兜底 | 高频伺服追踪 |

**YOLO 不负责追踪，追踪不依赖 YOLO。** 二者通过 LOCK 阶段交接。

---

## 2. 五大阶段概览

```
SEARCH ───→ LOCK ───→ TRACK ───→ HOVER
  ↑          │         │  │       │
  │          │         │  │       │
  └──────────┴─────────┴──┴───────┘
             LOST (异常出口)
```

| 阶段 | 触发条件 | 频率 | NPU 使用 | 目标 |
|------|---------|------|----------|------|
| **SEARCH** | 系统启动 / 从 LOST 回退 | YOLO 0.5Hz | **有** | 发现气球 |
| **LOCK** | YOLO 首次确认检测 | 一次性 (~2s) | 无 | 颜色建模 + 居中 |
| **TRACK** | LOCK 完成 | 25-30fps | **无** | 追踪 + 逼近 |
| **HOVER** | scale ≈ target 稳定 | 25-30fps | **无** | 悬停保持 |
| **LOST** | 连续 N 帧追踪失败 | 25-30fps | **无** | 恢复或回退 |

---

## 3. 阶段一: 目标发现 SEARCH

### 3.1 触发

- 系统启动
- LOST 阶段超时回退

### 3.2 帧采集

```
频率: 相机持续 30fps 采集 (NV12, V4L2 MPLANE)
YOLO 输入: 每 15 帧取 1 帧 (30fps / 2Hz = 15)
其余帧: 丢弃
```

### 3.3 YOLO 推理

```
步骤 1: 帧 resize 到 320×320
步骤 2: letterbox (保持宽高比, 灰边补齐)
步骤 3: rknn_inputs_set() → rknn_run() → rknn_outputs_get()
步骤 4: 解码输出 (YOLOv8 格式: cx, cy, w, h, conf per anchor)
步骤 5: NMS (IoU=0.45, conf_thresh=0.25)
步骤 6: 过滤: 只保留 "balloon" 类 (class_id=0)
```

### 3.4 检测确认

```
if 最新检测置信度 > 0.5:
    连续 3 次 YOLO 结果中 ≥2 次检测到:
        进入 LOCK
else:
    继续 SEARCH, 配合 yaw_rate 旋转
```

### 3.5 配合控制

此阶段控制层输出 (由状态机配置):
```
yaw_rate = 20.0 deg/s   // 固定旋转扫描
vx=0, vy=0, vz=0        // 不前进
```

---

## 4. 阶段二: 目标锁定 LOCK

### 4.1 姿态稳定

```
控制层: 零速度悬停 (vx=0, vy=0, vz=0, yaw_rate=0)
等待条件: 陀螺仪角速度 norm < 0.05 rad/s 持续 0.5s
超时: 最多等待 2s
```

### 4.2 确认检测

```
取最近 3 帧 YOLO 结果:
    有效检测帧数 ≥ 2
    AND bbox 中心位置波动 < 15% 画面宽度
    → 锁定成功
否则:
    → 误检, 回到 SEARCH
```

### 4.3 自适应颜色采样

#### 4.3.1 裁剪 bbox

```cpp
cv::Mat bbox_region = frame(bbox_rect);
cv::cvtColor(bbox_region, bbox_hsv, cv::COLOR_BGR2HSV);
```

#### 4.3.2 划分核心区

```
bbox 分区:
    margin = 20% (每边丢弃 20%, 四角必然是背景)

    ┌──────────────────────────────────┐  ← bbox 全区域
    │  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░   │
    │  ░░┌──────────────────────┐░░   │
    │  ░░│                      │░░   │  ← margin (丢弃)
    │  ░░│   core_region        │░░   │
    │  ░░│   (中央 60%)          │░░   │  ← 采样区域
    │  ░░│                      │░░   │
    │  ░░└──────────────────────┘░░   │
    │  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░   │
    └──────────────────────────────────┘

    int margin_x = bbox.width  * 0.20;
    int margin_y = bbox.height * 0.20;
    cv::Rect core_rect(
        bbox_rect.x + margin_x,
        bbox_rect.y + margin_y,
        bbox_rect.width  - 2 * margin_x,
        bbox_rect.height - 2 * margin_y
    );
    cv::Mat core_hsv = bbox_hsv(core_rect);
```

#### 4.3.3 构建 H-S 二维直方图

```cpp
// 只用 H 和 S 通道 (V 通道对光照变化敏感, 不使用)
int channels[] = {0, 1};       // H=0, S=1
int histSize[] = {30, 32};     // 粗量化 (对光照变化更鲁棒)
float h_ranges[] = {0, 180};   // H: 0-180 (OpenCV HSV 约定)
float s_ranges[] = {0, 256};   // S: 0-255
const float* ranges[] = {h_ranges, s_ranges};

cv::Mat hist;
cv::calcHist(&core_hsv, 1, channels, cv::Mat(), hist, 2, histSize, ranges);
cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
```

#### 4.3.4 提取自适应阈值参数

```cpp
// 从核心区域统计颜色分布
std::vector<cv::Mat> hsv_channels;
cv::split(core_hsv, hsv_channels);

// S 通道: 95% 分位值作为高饱和度参考
// (气球通常是高饱和度的, 95% 分位过滤掉噪声)
cv::Mat s_sorted;
cv::sort(hsv_channels[1].reshape(1,1), s_sorted, cv::SORT_EVERY_ROW + cv::SORT_ASCENDING);
int s_95th = s_sorted.at<uchar>(0, (int)(s_sorted.cols * 0.95));

// V 通道: 10% 分位值作为最低亮度
// (排除最暗的 10% — 阴影/边缘像素)
cv::Mat v_sorted;
cv::sort(hsv_channels[2].reshape(1,1), v_sorted, cv::SORT_EVERY_ROW + cv::SORT_ASCENDING);
int v_10th = v_sorted.at<uchar>(0, (int)(v_sorted.cols * 0.10));

// 这些值在后续反向投影时作为辅助阈值
// 用途: 在 calcBackProject 得到的概率图中,
//       对 S < s_95th*0.3 或 V < v_10th 的像素直接置零
```

### 4.4 计算参考面积 A₀

#### 4.4.1 第一次反向投影 (在 bbox 内)

```cpp
// 对 bbox 区域做反向投影
cv::Mat bbox_hsv_full = frame_hsv(bbox_rect);
cv::Mat backproj;
cv::calcBackProject(&bbox_hsv_full, 1, channels, hist, backproj, ranges);

// 辅助过滤: S 太低或 V 太低的像素置零
for (int y = 0; y < backproj.rows; y++) {
    for (int x = 0; x < backproj.cols; x++) {
        uchar s = bbox_hsv_full.at<cv::Vec3b>(y, x)[1];
        uchar v = bbox_hsv_full.at<cv::Vec3b>(y, x)[2];
        if (s < s_95th * 0.3 || v < v_10th) {
            backproj.at<uchar>(y, x) = 0;
        }
    }
}
```

#### 4.4.2 阈值化 + 形态学

```cpp
// 自适应阈值: 取反向投影最大值的 60%
double max_val;
cv::minMaxLoc(backproj, nullptr, &max_val);
int thresh = std::max(30, (int)(max_val * 0.6));

cv::Mat mask;
cv::threshold(backproj, mask, thresh, 255, cv::THRESH_BINARY);

// 形态学: 开运算去噪 + 闭运算填孔
cv::Mat kernel_3x3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
cv::Mat kernel_5x5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_3x3);
cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_5x5);
```

#### 4.4.3 找气球轮廓

```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// 选面积最大的轮廓
double max_area = 0;
int best_idx = -1;
for (size_t i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);
    // 面积必须合理: bbox 面积的 15% ~ 85%
    double bbox_area = bbox_rect.width * bbox_rect.height;
    if (area > bbox_area * 0.15 && area < bbox_area * 0.85 && area > max_area) {
        max_area = area;
        best_idx = i;
    }
}

if (best_idx < 0) {
    // 找不到合格轮廓, 缩小核心区到 50%, 重试
    // (可能 bbox 太大了, 包含了太多背景)
    goto 4.3.2 用 50% 核心区重试;
}
```

#### 4.4.4 圆度验证

```cpp
double area = cv::contourArea(contours[best_idx]);
double perimeter = cv::arcLength(contours[best_idx], true);
double circularity = 4 * CV_PI * area / (perimeter * perimeter);
// 完美圆 = 1.0, 方形 ≈ 0.785

if (circularity < 0.6) {
    // 轮廓不够圆 — 可能 bbox 仍然太大, 包含非气球区域
    // 减小核心区到 50%, 重新采样 + 重新反向投影
    goto 4.3.2 用 50% 核心区重试;
}
```

#### 4.4.5 计算最小外接圆

```cpp
cv::Point2f center;
float radius;
cv::minEnclosingCircle(contours[best_idx], center, radius);

double A_frame = CV_PI * radius * radius;
```

#### 4.4.6 多帧取中位数

```cpp
// 连续取 3 帧的 A_frame, 取中位数作为 A₀
float A_samples[3];
for (int i = 0; i < 3; i++) {
    A_samples[i] = compute_A_from_current_frame();  // 重复 4.4.1-4.4.5
    // 帧间等待 ~33ms (下一帧)
}
std::sort(A_samples, A_samples + 3);
float A0 = A_samples[1];  // 中位数
```

### 4.5 视觉居中

#### 4.5.1 计算误差

```cpp
float x_error = (balloon_cx - image_cx) / (float)image_width;   // [-0.5, 0.5]
float y_error = (balloon_cy - image_cy) / (float)image_height;  // [-0.5, 0.5]

// balloon_cx, balloon_cy = minEnclosingCircle 的圆心
// image_cx = frame_width / 2
// image_cy = frame_height / 2
```

#### 4.5.2 控制指令

```
yaw_rate = K_yaw × x_error    (左右旋转, 让气球水平居中)
vz       = K_alt × y_error    (上下平移, 让气球垂直居中)
                               y_error > 0: 气球偏下 → 无人机下降
                               y_error < 0: 气球偏上 → 无人机上升
```

#### 4.5.3 退出条件

```
| x_error | < 0.05 AND | y_error | < 0.05    持续 ≥ 0.5s (≈15帧)
→ 进入 TRACK
```

---

## 5. 阶段三: 高频追踪 TRACK

这是系统运行时间最长的阶段 (90%+), 每帧执行。

### 5.1 卡尔曼预测

#### 5.1.1 状态定义

```
X = [cx, cy, vx_img, vy_img, radius]^T

cx, cy:     气球中心在图像中的像素坐标
vx_img:     cx 的变化速率 (像素/帧)
vy_img:     cy 的变化速率 (像素/帧)
radius:     气球最小外接圆半径 (像素)
```

#### 5.1.2 预测方程

```cpp
const float dt = 1.0f;  // 归一化到 1 帧

// 状态转移矩阵 (恒速模型)
cv::Mat F = (cv::Mat_<float>(5,5) <<
    1, 0, dt, 0,  0,
    0, 1, 0,  dt, 0,
    0, 0, 1,  0,  0,
    0, 0, 0,  1,  0,
    0, 0, 0,  0,  1
);

// 预测
cv::Mat X_pred = F * X;
float pred_cx = X_pred.at<float>(0);
float pred_cy = X_pred.at<float>(1);
float pred_radius = X_pred.at<float>(4);
```

#### 5.1.3 搜索窗口

```cpp
float half_side = pred_radius * 2.5f;   // 自适应大小
half_side = std::max(half_side, 30.0f);  // 最小 30px
half_side = std::min(half_side, 300.0f); // 最大 300px

cv::Rect search_roi(
    std::max(0, (int)(pred_cx - half_side)),
    std::max(0, (int)(pred_cy - half_side)),
    std::min(frame_width  - (int)(pred_cx - half_side), (int)(2 * half_side)),
    std::min(frame_height - (int)(pred_cy - half_side), (int)(2 * half_side))
);
```

### 5.2 局部反向投影

```cpp
cv::Mat search_roi_hsv = frame_hsv(search_roi);
cv::Mat backproj;
cv::calcBackProject(&search_roi_hsv, 1, channels, hist, backproj, ranges);

// 自适应阈值
double max_val;
cv::minMaxLoc(backproj, nullptr, &max_val);
int thresh = std::max(25, (int)(max_val * 0.5));  // 比 LOCK 阶段略低, 因为搜索窗口更精确

cv::threshold(backproj, mask, thresh, 255, cv::THRESH_BINARY);
```

### 5.3 形态学清理

```cpp
cv::Mat kernel_3x3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
cv::Mat kernel_5x5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_3x3);   // 去噪点
cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_5x5);  // 填小孔

// 如果 mask 中白色像素太少, 跳过此帧
if (cv::countNonZero(mask) < 10) {
    lost_counter++;
    return;  // 等待下一帧
}
```

### 5.4 多条件加权打分

```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

struct ContourScore { int idx; float score; };
std::vector<ContourScore> scores;

for (size_t i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);
    double perimeter = cv::arcLength(contours[i], true);

    // ---- 条件 1: 圆度 (权重 0.35) ----
    float circularity = (perimeter > 0)
        ? 4 * CV_PI * area / (perimeter * perimeter) : 0;
    float score_circ = std::min(circularity, 1.0f);

    // ---- 条件 2: 面积匹配度 (权重 0.30) ----
    float expected_area = CV_PI * pred_radius * pred_radius;
    float score_area = 1.0f - std::abs(area - expected_area)
                        / std::max(area, (double)expected_area);

    // ---- 条件 3: 位置匹配度 (权重 0.20) ----
    cv::Moments m = cv::moments(contours[i]);
    float cx = m.m10 / m.m00;
    float cy = m.m01 / m.m00;
    float dist = std::sqrt((cx - pred_cx)*(cx - pred_cx)
                          + (cy - pred_cy)*(cy - pred_cy));
    float score_pos = 1.0f - std::min(dist / half_side, 1.0f);

    // ---- 条件 4: 颜色一致性 (权重 0.15) ----
    cv::Mat contour_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::drawContours(contour_mask, contours, i, 255, cv::FILLED);
    float mean_color = (float)cv::mean(backproj, contour_mask)[0] / 255.0f;
    float score_color = mean_color;

    // ---- 综合得分 ----
    float total = 0.35f * score_circ + 0.30f * score_area
                + 0.20f * score_pos + 0.15f * score_color;

    scores.push_back({(int)i, total});
}

// 选得分最高的
std::sort(scores.begin(), scores.end(),
          [](auto& a, auto& b) { return a.score > b.score; });

if (scores.empty() || scores[0].score < 0.5f) {
    lost_counter++;
    return;
}

best_contour = contours[scores[0].idx];
lost_counter = 0;  // 重置
```

**为什么打分制优于硬阈值**:

| | 硬阈值 | 加权打分 |
|---|---|---|
| 决策边界 | 陡峭 (0.601=过, 0.599=不过) | 平滑 (得分渐变) |
| 多条件组合 | 全部必须过 → 可能漏检 | 综合评估 → 鲁棒 |
| 临界情况 | 浮点误差导致抖动 | 得分连续, 不抖动 |
| 参数敏感度 | 高 | 低 |

### 5.5 最小外接圆 + 面积比

```cpp
cv::Point2f center;
float radius;
cv::minEnclosingCircle(best_contour, center, radius);

float A_current = CV_PI * radius * radius;

float scale_factor = std::sqrt(A0 / A_current);
// A0 是参考帧面积 (LOCK 阶段保存的)
// scale_factor: 1.0=参考距离, <1=更近, >1=更远
```

### 5.6 卡尔曼更新

```cpp
// 观测值
cv::Mat Z = (cv::Mat_<float>(5,1) << center.x, center.y, 0, 0, radius);

// 卡尔曼增益
cv::Mat K = P_pred * H.t() * (H * P_pred * H.t() + R).inv();

// 状态更新
X = X_pred + K * (Z - H * X_pred);

// 协方差更新
P = (I - K * H) * P_pred;
```

### 5.7 A₀ 在线递推更新

每 30 帧 (≈1 秒) 执行一次，仅在追踪质量高时:

```cpp
if (frame_count % 30 == 0 && best_score > 0.8 && circularity > 0.85) {
    // 把当前 A 换算回参考距离的值
    float scale_drift = std::sqrt(A0 / A_current);
    // A0_new = α × A0_old + (1-α) × A_current_normalized_to_ref_distance
    A0 = 0.9f * A0 + 0.1f * A_current * scale_drift * scale_drift;

    // α=0.9 → 保留 90% 旧信息, 只吸收 10% 新信息 → 平滑过渡
}
```

**为什么需要递推更新**: 随着无人机靠近, 气球像素增多, 轮廓提取更精确。用后续的精确数据修正早期的粗糙 A₀。

### 5.8 视觉伺服控制输出

```cpp
// 计算误差
float x_error = (center.x - image_cx) / (float)image_width;
float y_error = (center.y - image_cy) / (float)image_height;

// 速度指令 (图像空间 → 机体速度)
float yaw_rate  = K_YAW  * x_error;
float vz        = K_ALT  * y_error;
float vx        = K_VX   * std::log(scale_factor / TARGET_SCALE);

// 侧向修正: 只有水平偏差大时才侧移
float vy = 0;
if (std::abs(x_error) > 0.2f) {
    vy = K_SIDE * x_error;
}

// Clamp
vx = std::clamp(vx, -MAX_SPEED_MS, MAX_SPEED_MS);
vy = std::clamp(vy, -MAX_SPEED_MS, MAX_SPEED_MS);
vz = std::clamp(vz, -MAX_SPEED_MS, MAX_SPEED_MS);
```

**为什么用对数映射**:

| scale | 含义 | 线性 vx = (scale-target) × 5 | 对数 vx = log(scale/target) × 5 |
|-------|------|-----|------|
| 2.0 | 太近 | +7.5 (猛退) | +3.5 (温和后退) |
| 1.0 | 参考距离 | +2.5 (前进) | 0 (不动) |
| 0.5 | 目标 ✅ | 0 (不动) | 0 (不动) |
| 0.3 | 太远 | -1.0 (慢接近) | -4.1 (加速接近) |

对数方案在"太远"时加速追赶, 在"太近"时温和后退, 更符合直觉。

### 5.9 过渡判断

```cpp
// APPROACH → HOVER
if (std::abs(scale_factor - TARGET_SCALE) < EPSILON_SCALE &&
    std::abs(x_error) < EPSILON_X &&
    std::abs(y_error) < EPSILON_Y) {
    stable_frames++;
    if (stable_frames > 30) {  // 持续 1 秒
        transition_to(HOVER);
    }
} else {
    stable_frames = 0;
}
```

---

## 6. 阶段四: 目标保持 HOVER

### 6.1 进入条件

```
|scale - target| < ε  AND  |x_error| < ε_x  AND  |y_error| < ε_y
    持续 ≥ 30 帧
```

### 6.2 控制指令

```
速度指令改为纯补偿模式:
    yaw_rate = K_YAW × x_error    (保持水平居中)
    vz       = K_ALT × y_error    (保持垂直居中)
    vx       = 0                  ← 不再前进
    vy       = 0                  ← 不再侧移
```

### 6.3 漂移检测

每 30 帧检查一次:

```cpp
if (std::abs(scale_factor - TARGET_SCALE) > 2 * EPSILON_SCALE) {
    // 气球飘远了或飘近了
    transition_to(TRACK);  // 回到追踪/逼近
}
```

### 6.4 追踪继续

HOVER 阶段**仍然每帧执行整个 5.1-5.7 追踪流程**, 只是控制输出改为零速补偿。这样气球一旦开始飘动可以立刻检测到。

---

## 7. 阶段五: 异常恢复 LOST

### 7.1 进入条件

```
lost_counter ≥ 5  进入"悬停等待"
lost_counter ≥ 30 进入 LOST
```

### 7.2 单帧干扰容忍 (lost_counter < 5)

```
lost_counter: 0 → 1 → 0 → 1 → 0 (被正常帧不断重置)
→ 无任何影响
```

### 7.3 短暂丢失 (5 ≤ lost_counter < 30)

```
控制层: 零速度悬停 (vx=0, vy=0, vz=0, yaw_rate=0)
视觉:   不更新卡尔曼预测, 搜索窗口停在最后已知位置
        每帧仍然做反向投影 + 轮廓搜索
        如果某帧恢复: lost_counter = 0, 恢复正常追踪
```

**为什么不是 0 就变 LOST**: 短暂遮挡 (气球飞过树枝 0.1s、逆光 2 帧、电线遮挡) 是正常的。等待 1s 比立即回退到 YOLO 高效得多。

### 7.4 长期丢失 (lost_counter ≥ 30)

```
步骤 1: 宣布 LOST
步骤 2: 控制层发送零速度悬停
步骤 3: 等待 3 秒 (在此期间颜色追踪继续尝试恢复)
步骤 4: 如果恢复 (任意帧 score > 0.5):
            → 回到 TRACK
步骤 5: 如果 3 秒后仍未恢复:
            → 清空直方图
            → 清空 A₀
            → 清空卡尔曼状态
            → 回到 SEARCH (重新启用 YOLO 0.5Hz)
```

### 7.5 追踪质量降级检测 (预防性降级)

```cpp
// 每 100 帧统计一次平均得分
if (frame_count % 100 == 0) {
    float avg_score = score_accumulator / 100.0f;
    score_accumulator = 0;

    if (avg_score < 0.6f) {
        // 追踪质量在持续降低 (但还没完全丢失)
        // → 主动触发一次 YOLO 检测
        trigger_yolo_check();

        if (yolo_also_fails) {
            transition_to(LOST);
        } else {
            // YOLO 检测到了 → 说明光照/颜色变化导致追踪退化
            // → 重新 LOCK (用 YOLO bbox 重新采样颜色)
            transition_to(LOCK);
        }
    }
}
```

**这是对光照变化的工程化防御**: 在追踪还没彻底失败之前就主动用 YOLO 重校验。

---

## 8. 关键数据结构

```cpp
// ============================================================
// 颜色模型
// ============================================================
struct ColorModel {
    cv::Mat h_s_histogram;        // 30×32 H-S 二维直方图
    int     s_threshold;          // 饱和度最低阈值
    int     v_threshold;          // 亮度最低阈值
    bool    valid;
};

// ============================================================
// 参考帧信息
// ============================================================
struct ReferenceFrame {
    float   A0;                   // 参考最小外接圆面积
    cv::Point2f image_center;     // 参考帧时气球的图像中心
    ColorModel color_model;
    bool    valid;
};

// ============================================================
// 追踪结果 (每帧输出)
// ============================================================
struct TrackingResult {
    // 气球在图像中的状态
    cv::Point2f center;           // 最小外接圆中心 (像素坐标)
    float       radius;           // 最小外接圆半径 (像素)
    float       A;                // 当前最小外接圆面积
    float       circularity;      // 圆度 (1.0=完美圆)

    // 面积比 (核心控制变量)
    float       scale_factor;     // sqrt(A0 / A)

    // 图像空间误差
    float       x_error;          // 归一化 [-0.5, 0.5]
    float       y_error;          // 归一化 [-0.5, 0.5]

    // 质量
    float       best_score;       // 轮廓综合得分
    bool        valid;            // 本帧追踪成功

    // 控制指令
    float       cmd_yaw_rate;
    float       cmd_vx;
    float       cmd_vy;
    float       cmd_vz;

    // 元信息
    int         lost_counter;
    uint64_t    timestamp_us;
};

// ============================================================
// 视觉识别阶段枚举
// ============================================================
enum class VisionPhase : uint8_t {
    SEARCH   = 0,   // YOLO 搜索
    LOCK     = 1,   // 颜色采样 + 居中
    TRACK    = 2,   // 高频追踪 + 逼近
    HOVER    = 3,   // 悬停保持
    LOST     = 4,   // 丢失恢复
};

// ============================================================
// YOLO 检测结果
// ============================================================
struct YoloDetection {
    cv::Rect bbox;
    float    confidence;
    bool     valid;
    uint64_t timestamp_us;
};

// ============================================================
// 视觉模块主接口输出
// ============================================================
struct VisionOutput {
    VisionPhase    phase;
    YoloDetection  yolo_result;    // SEARCH 阶段有效
    TrackingResult track_result;   // TRACK/HOVER 阶段有效
    ColorModel     color_model;    // LOCK 阶段有效
};
```

---

## 9. OpenCV 核心操作详解

### 9.1 反向投影 `calcBackProject`

**作用**: 对输入图像的每个像素, 查它在直方图中的值, 输出"这个像素属于目标颜色的概率"。

```
输入:  search_roi_hsv (搜索窗口 HSV 图)
       hist (LOCK 阶段建立的气球 H-S 直方图)

输出:  backproj (灰度图, 每个像素 ∈ [0,255])
       255: 这个像素的 (H,S) 值在直方图中有很高的计数 → 很可能是气球
       0:   这个像素的 (H,S) 值在直方图中没有出现 → 不是气球

工作原理:
    对于像素 p = (H=5, S=180):
        查 hist[5][180] = 200 (直方图计数)
        backproj[p] = 200 / 255 = 0.78 → 这个像素有 78% 的概率属于气球
```

### 9.2 最小外接圆 `minEnclosingCircle`

**作用**: 找到一个能完全包含给定轮廓的最小圆。

```
输入:  任意形状的轮廓点集
输出:  (center.x, center.y), radius

示例:
    轮廓: 水滴形 (宽 80px, 高 100px, 底部尖锐)
    minEnclosingCircle → center=(40,45), radius=40

    圆包含整个轮廓, 直径 = 轮廓最宽处 = 80px
    即使气球被拉长到 100px 高, 圆半径仍然是 40px

    原始轮廓面积: ~3500 px²  (随姿态波动 ±30%)
    外接圆面积:   ~5027 px²  (始终由最宽处决定, 波动 <5%)
```

### 9.3 圆度公式 `4πA/P²`

**作用**: 衡量一个轮廓有多"圆"。完美圆 = 1.0。

```
推导:
    对于半径为 r 的完美圆: A = πr², P = 2πr
    4πA/P² = 4π(πr²)/(4π²r²) = 1.0

    对于正方形 (边长 s): A = s², P = 4s
    4πA/P² = 4πs²/16s² = π/4 ≈ 0.785

    对于细长椭圆 (长轴 2r, 短轴 r): A = 2πr², P ≈ 2π√((4r²+r²)/2) ≈ 2πr√2.5
    4πA/P² ≈ 4π×2πr²/(4π²×2.5r²) = 2/2.5 = 0.8

我们设置阈值 0.6 是相当宽松的, 足以容纳气球轻微变形。
```

---

## 10. 颜色模型: H-S 直方图 vs 固定阈值

### 10.1 固定 HSV 区间的问题

```
红色气球的 HSV 固定区间:
H ∈ [0, 10] ∪ [170, 180], S ∈ [150, 255], V ∈ [50, 255]

问题:
1. 红色在 H 空间被截断 (0 和 180 是同一个颜色, 但区间跨越两端)
2. 光照变化后 S/V 区间不准确
3. 区间外的"红色"被漏掉, 区间内的"非红色"被误判
```

### 10.2 直方图的优势

```
H-S 直方图 (30×32 格):

    S →
  H  ┌─────────────────────────────────┐
  ↓  │ ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  │
     │ ·  ·  ·  ·  ██ ·  ·  ·  ·  ·  │
     │ ·  ·  ·  ·  ████·  ·  ·  ·  ·  │  峰值在 (H=2~8, S=180~220)
     │ ·  ·  ·  ·  ███████·  ·  ·  ·  │  代表"这个气球在当前光照下的红色"
     │ ·  ·  ·  ·  ████·  ·  ·  ·  ·  │
     │ ·  ·  ·  ·  ██ ·  ·  ·  ·  ·  │  不是矩形框, 而是一个渐变的概率山峰
     │ ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  │
     └─────────────────────────────────┘

特点:
1. 不需要手动设置 H 区间 — 直方图自动找到峰值
2. 自动处理 H 环绕 — 如果气球恰好在红色边界 (H≈0/180), 直方图两端都有计数
3. 光照变化自适应 — 每次 LOCK 重新采样, 始终匹配当前光照
4. 概率化 — 不是硬"是/否", 而是"像素有 78% 概率属于气球"
```

---

## 11. 面积比控制原理

### 11.1 数学基础

小孔成像模型:

```
物体在图像上的面积 ∝ 1 / 距离²

即: A = k / d², 其中 k = 物体真实面积 × 焦距²

对于两次观测 (参考帧和当前帧):
A₀ = k / d₀²
A  = k / d²

相除:
A₀ / A = (d / d₀)²

开方:
√(A₀ / A) = d / d₀ = scale_factor
```

k 和 d₀ 都在除法中消去了。**你不需要知道焦距、不需要知道气球直径、不需要知道参考距离。** 你只需要知道"我飞到了参考距离的 0.5 倍"。

### 11.2 控制策略

```
设 TARGET_SCALE = 0.5 (飞到参考距离的一半)

控制律: vx = K × log(scale / target)

分析:
    scale = 0.5 (正好在目标距离) → log(1) = 0 → vx = 0 ✓
    scale = 1.0 (在参考距离)    → log(2) = 0.69 → vx = +0.69K (前进)
    scale = 2.0 (太近, 参考距 2 倍)= log(4) = 1.39 → vx = +1.39K (后退... 等等不对)

    让我重新检查: scale = 1.0 时在参考距离, scale = 0.5 时在目标距离 (更近)
    scale 更大 = 更近 → 需要后退
    scale 更小 = 更远 → 需要前进

    scale = 0.5 (正好) → log(1) = 0 → vx = 0 ✓ (不动)
    scale = 1.0 (在参考距离)→ log(2) = 0.69 → 前进 ✓ (要飞到 0.5)
    scale = 2.0 (太近!) → log(4) = 1.39 → 但是应该后退!

    问题: scale > target 时 log 正的更多, 但我们需要后退 (负 vx)

修正:
    vx = -K × log(scale / target) × MAX_SPEED
    或更直接:
    vx = K × (target - scale) / target 当 scale > target 时 vx < 0 (后退)
```

实际上最直观的表示是:

```cpp
// 误差 = (当前 scale - 目标 scale) / 目标 scale
// >0: 太近 → 后退 (vx < 0)
// <0: 太远 → 前进 (vx > 0)
// =0: 正好 → 不动

float scale_error = (scale_factor - TARGET_SCALE) / TARGET_SCALE;

// 对数映射: 误差大时增益大, 误差小时增益小
vx = -K_VX * std::copysign(std::log(1.0f + std::abs(scale_error)),
                            scale_error);

// clamp
vx = std::clamp(vx, -MAX_SPEED_MS, MAX_SPEED_MS);
```

### 11.3 为什么不需要"飞到 N 米"

| 需要 | 绝对距离方案 | 面积比方案 |
|------|-------------|-----------|
| 相机标定 (fx, fy) | 必须 | 不需要 |
| 气球物理直径 | 必须预估 | 不需要 |
| 无人机高度 | 必须 (地面假设) | 不需要 |
| GPS | 需要 | 不需要 |
| 标定精度维持 | 持续要求 | 不需要 |

气球的直径是 30cm 还是 1m 都不影响面积比的正确性——因为同一个气球在距离变化时, 面积比只和距离比有关。

---

## 12. 卡尔曼预测器

### 12.1 为什么需要预测

场景: 气球在飘动 (随风), 无人机同时在运动 (伺服追踪)。

```
如果只是用上一帧的中心作为这一帧的搜索中心:
    搜索窗口中心落后于气球真实位置
    → 下一帧气球可能跑到搜索窗口边缘甚至外面
    → 追踪失败率增加

用卡尔曼预测"下一帧会在哪里":
    考虑了运动速度
    → 搜索窗口提前偏移
    → 气球在窗口中心附近
    → 追踪成功率大幅提高
```

### 12.2 模型

```
状态变量 (5维):
    X = [cx, cy, vx, vy, radius]

状态转移 (恒速模型):
    cx_{t+1} = cx_t + vx_t × dt
    cy_{t+1} = cy_t + vy_t × dt
    vx_{t+1} = vx_t  (+ noise)
    vy_{t+1} = vy_t  (+ noise)
    radius_{t+1} = radius_t  (+ noise, 距离不变时 radius 近似不变)

观测 (5维):
    Z = [cx, cy, 0, 0, radius]  (只能观测位置和大小, 不能直接观测速度)
```

### 12.3 初始化

```cpp
cv::KalmanFilter kf(5, 5, 0);  // 5 状态, 5 观测

// 状态转移矩阵
kf.transitionMatrix = (cv::Mat_<float>(5,5) <<
    1, 0, 1, 0, 0,   // cx += vx
    0, 1, 0, 1, 0,   // cy += vy
    0, 0, 1, 0, 0,   // vx 不变
    0, 0, 0, 1, 0,   // vy 不变
    0, 0, 0, 0, 1);  // radius 不变

// 观测矩阵 (观测 = 状态, 但看不到速度)
kf.measurementMatrix = cv::Mat::eye(5, 5, CV_32F);

// 过程噪声协方差 (Q)
// 位置噪声小, 速度噪声大 (速度变化不可预测)
float Q_vals[] = {1e-2, 0,    0,    0,    0,
                  0,    1e-2, 0,    0,    0,
                  0,    0,    5e-1, 0,    0,   // vx 噪声大
                  0,    0,    0,    5e-1, 0,   // vy 噪声大
                  0,    0,    0,    0,    1e-2};
kf.processNoiseCov = cv::Mat(5, 5, CV_32F, Q_vals);

// 测量噪声协方差 (R)
float R_vals[] = {1e-3, 0,    0,    0,    0,
                  0,    1e-3, 0,    0,    0,
                  0,    0,    1,    0,    0,  // 速度不可直接观测
                  0,    0,    0,    1,    0,
                  0,    0,    0,    0,    2e-2};
kf.measurementNoiseCov = cv::Mat(5, 5, CV_32F, R_vals);

// 初始状态: 使用 LOCK 阶段的 center 和 radius
kf.statePost.at<float>(0) = initial_cx;
kf.statePost.at<float>(1) = initial_cy;
kf.statePost.at<float>(2) = 0;   // 初始速度未知
kf.statePost.at<float>(3) = 0;
kf.statePost.at<float>(4) = initial_radius;
```

---

## 13. 多阶段控制指令映射

各阶段对 `ControlCommand` 的填充:

```
┌──────────┬──────────────────────────────────────────────────┐
│  阶段    │ 控制输出                                         │
├──────────┼──────────────────────────────────────────────────┤
│ SEARCH   │ yaw_rate = SEARCH_YAW_RATE (20°/s)               │
│          │ vx=0, vy=0, vz=0                                 │
├──────────┼──────────────────────────────────────────────────┤
│ LOCK     │ (稳定) vx=0, vy=0, vz=0, yaw_rate=0              │
│          │ (居中) yaw_rate = K_YAW × x_error                │
│          │         vz       = K_ALT × y_error               │
│          │         vx=0, vy=0                               │
├──────────┼──────────────────────────────────────────────────┤
│ TRACK    │ yaw_rate = K_YAW × x_error                       │
│          │ vz       = K_ALT × y_error                       │
│          │ vx       = (面积比控制)                           │
│          │ vy       = (如果偏差大) K_SIDE × x_error          │
├──────────┼──────────────────────────────────────────────────┤
│ HOVER    │ yaw_rate = K_YAW × x_error  (仅补偿)             │
│          │ vz       = K_ALT × y_error  (仅补偿)             │
│          │ vx=0, vy=0                                       │
├──────────┼──────────────────────────────────────────────────┤
│ LOST     │ (等待时) 全零                                     │
│          │ (回退后) 同 SEARCH                               │
└──────────┴──────────────────────────────────────────────────┘
```

---

## 14. 参数配置表

```yaml
# ============================================================
# YOLO 检测参数
# ============================================================
yolo:
  model_path: "model/balloon_int8.rknn"
  input_size: [320, 320]
  confidence_threshold: 0.25
  nms_threshold: 0.45
  detection_interval_frames: 15    # 30fps / 15 = 0.5Hz
  lock_confirm_frames: 3           # 连续 N 帧中 ≥2 帧检测到
  lock_confirm_ratio: 0.67         # 确认比例 = 2/3

# ============================================================
# LOCK 阶段参数
# ============================================================
lock:
  stabilize_timeout_s: 2.0         # 姿态稳定超时
  core_region_ratio: 0.6           # 核心采样区占比
  core_region_fallback: 0.5        # 第一次失败后缩小到 50%
  min_circularity: 0.6             # 最小圆度
  contour_area_min_ratio: 0.15     # 轮廓面积 >= bbox 的 15%
  contour_area_max_ratio: 0.85     # 轮廓面积 <= bbox 的 85%
  A0_sample_frames: 3              # A₀ 采样帧数

# ============================================================
# 直方图参数
# ============================================================
histogram:
  h_bins: 30                       # H 通道量化级别
  s_bins: 32                       # S 通道量化级别
  backproj_threshold_ratio: 0.5    # 阈值 = max * 0.5
  s_percentile: 0.95               # S 通道 95% 分位
  v_percentile: 0.10               # V 通道 10% 分位

# ============================================================
# TRACK 阶段参数
# ============================================================
track:
  search_window_scale: 2.5         # 搜索窗口 = radius * 2.5
  search_window_min: 30            # 最小搜索窗口 (px)
  search_window_max: 300           # 最大搜索窗口 (px)
  min_mask_pixels: 10              # mask 最少白色像素

  # 轮廓打分权重 (总和=1.0)
  score_weight_circularity: 0.35
  score_weight_area: 0.30
  score_weight_position: 0.20
  score_weight_color: 0.15
  min_score: 0.5                   # 最低综合得分

  # A₀ 更新
  A0_update_interval_frames: 30    # 每 30 帧更新一次
  A0_update_alpha: 0.9             # EMA 平滑系数
  A0_update_min_score: 0.8         # 更新条件: 得分 > 0.8
  A0_update_min_circularity: 0.85  # 更新条件: 圆度 > 0.85

  # 形态学
  morph_open_kernel: 3
  morph_close_kernel: 5

# ============================================================
# 面积比控制参数
# ============================================================
scale_control:
  target_scale: 0.5                # 目标: 飞到参考距离的一半
  epsilon_scale: 0.05              # 到达判断的容差

# ============================================================
# 视觉伺服增益
# ============================================================
servo:
  K_YAW: 2.0                       # 偏航角速度增益 (rad/s per unit error)
  K_ALT: 3.0                       # 高度速度增益 (m/s per unit error)
  K_VX: 2.0                        # 前进速度增益
  K_SIDE: 1.5                      # 侧移速度增益
  max_speed_ms: 5.0                # 最大速度
  centering_threshold: 0.05        # 居中判定阈值

# ============================================================
# LOST 阶段参数
# ============================================================
lost:
  brief_lost_frames: 5             # 开始悬停等待的阈值
  full_lost_frames: 30             # 宣布 LOST 的阈值
  wait_timeout_s: 3.0              # LOST 后等待恢复的时间
  quality_check_interval: 100      # 追踪质量检查间隔 (帧)
  quality_min_avg_score: 0.6       # 预防性降级的最低平均得分

# ============================================================
# HOVER 阶段参数
# ============================================================
hover:
  stable_frames: 30                # 进入 HOVER 需要的稳定帧数
  drift_check_interval: 30         # 漂移检测间隔
  drift_threshold: 2.0             # scale 漂移阈值倍数
```

---

## 15. 完整流程图

```
                          ┌──────────────────┐
                          │      启动         │
                          └────────┬─────────┘
                                   │
                                   ▼
                     ┌─────────────────────────┐
                     │      SEARCH              │
                     │                          │
                     │  · 相机 30fps 采集        │
                     │  · 每 15 帧取 1 帧 YOLO   │
                     │  · 控制: yaw_rate 旋转    │
                     └────────────┬────────────┘
                                  │ 3帧中≥2帧检测到气球
                                  │ confidence > 0.5
                                  ▼
                     ┌─────────────────────────┐
                     │      LOCK                │
                     │                          │
                     │  · [0.5s] 姿态稳定        │
                     │  · 确认检测 (连续确认)    │
                     │  · ┌──────────────────┐  │
                     │  │ 自适应颜色采样      │  │
                     │  │ · 划分核心区 (60%)  │  │
                     │  │ · 构建 H-S 直方图   │  │
                     │  │ · 提取 S/V 阈值     │  │
                     │  └──────────────────┘  │
                     │  · ┌──────────────────┐  │
                     │  │ 计算 A₀            │  │
                     │  │ · 反向投影          │  │
                     │  │ · 形态学清理        │  │
                     │  │ · 轮廓 → 圆度验证   │  │
                     │  │ · minEnclosingCircle│  │
                     │  │ · 3 帧中位数 A₀     │  │
                     │  └──────────────────┘  │
                     │  · 视觉居中控制         │
                     └────────────┬────────────┘
                                  │ |x_error|<0.05 & |y_error|<0.05
                                  │ 持续 ≥0.5s
                                  ▼
          ┌───────────────────────────────────────────┐
          │               TRACK                        │
          │                                           │
          │  每帧 (~30fps):                            │
          │  ┌─────────────────────────────────────┐  │
          │  │ 卡尔曼预测 → 搜索窗口 (自适应大小)    │  │
          │  │ 局部反向投影 (H-S 直方图)            │  │
          │  │ 形态学 (开运算 + 闭运算)              │  │
          │  │ findContours → 多条件加权打分         │  │
          │  │ minEnclosingCircle → A, scale_factor │  │
          │  │ 卡尔曼更新                           │  │
          │  │ A₀ 在线递推更新 (每 30 帧)            │  │
          │  └─────────────────────────────────────┘  │
          │                                           │
          │  控制输出:                                 │
          │    yaw_rate = K_YAW × x_error              │
          │    vz       = K_ALT × y_error              │
          │    vx       = 面积比控制                   │
          │                                           │
          │  ┌──────── 每帧判断 ──────────────────┐   │
          │  │ score < 0.5 → lost_counter++        │   │
          │  │ lost ≥ 5 → 悬停等待                  │   │
          │  │ lost ≥ 30 → LOST                    │   │
          │  │ scale≈target 稳定 → HOVER           │   │
          │  │ 每 100 帧质量检查 → 预防性降级       │   │
          │  └─────────────────────────────────────┘   │
          └──────┬────────────────┬───────────────────┘
                 │                │
    scale≈target │                │ lost≥30
    稳定 30 帧   │                │ 3s 未恢复
                 ▼                ▼
          ┌──────────┐     ┌──────────┐
          │  HOVER   │     │  LOST    │
          │          │     │          │
          │ 零速度    │     │ 悬停 3s   │
          │ 仅补偿    │     │ 等待恢复  │
          │          │     │          │
          │ 漂移 →   │     │ 超时 →   │
          │ TRACK    │     │ SEARCH   │
          └──────────┘     └────┬─────┘
                                │
                                └──→ SEARCH (重新 YOLO)
```

---

## 16. 错误处理与边缘情况

### 16.1 相机帧丢失

```
现象: V4L2 poll() 返回错误或 dequeue 超时
处理:
    1. 重试 3 次 (每次等待 100ms)
    2. 仍失败 → 设置 camera_error = true
    3. 控制层发送零速度悬停
    4. 在后台尝试重新打开摄像头
```

### 16.2 NPU 推理超时/失败

```
现象: rknn_run() 返回非零 或 超过 100ms 未完成
处理:
    1. 记录错误日志
    2. 使用上次有效的 YOLO 结果 (如果有, < 5s 内的)
    3. 连续 3 次失败 → 重置 RKNN 上下文
    4. 仍失败 → 仅依赖颜色追踪 (如果已初始化)
```

### 16.3 气球遮挡

```
场景 1: 短暂遮挡 (< 0.17s = 5 帧, 如飞过树枝)
    → lost_counter < 5 → 无影响 → 自动恢复

场景 2: 中等遮挡 (< 1s = 30 帧, 如电线在画面中)
    → lost_counter ≥ 5 → 悬停等待
    → 颜色追踪持续尝试
    → 遮挡过去后自动恢复

场景 3: 完全遮挡 (> 1s, 如气球飞到建筑物后面)
    → lost_counter ≥ 30 → LOST
    → 3 秒等待 → 回退 SEARCH
```

### 16.4 光照剧变 (出云/逆光)

```
场景: 无人机从阴凉处飞到阳光下
    → 气球的 HSV 值整体偏移
    → 反向投影概率降低 (颜色模型过时)
    → 但轮廓可能仍能找到 (形状还在)

    预防: 每 100 帧质量检查
    → avg_score < 0.6
    → 触发 YOLO 重新检测
    → YOLO 不依赖颜色, 仍能检测
    → 重新 LOCK (采样新光照下的颜色)
```

### 16.5 同色背景接近气球 (最坏场景)

```
场景: 红色气球 + 红色圆形标志牌在同一个搜索窗口内
    → findContours 找到两个红色圆形轮廓
    → 按加权打分排序:
       得分最高的可能是气球 (大小/位置/颜色匹配度更高)
       第二个得分也可能 > 0.5
    → 选得分最高的 (大概率是气球)
    → 如果得分最高的不是气球 (误匹配):
       下一帧预测位置偏离 → 真实气球轮廓得分更高 → 切换回来
       最坏: 2-3 帧误匹配 → lost_counter 累积
       → 如果持续误匹配: 触发 YOLO 重校验 → YOLO 能正确区分
```

### 16.6 气球快速远离 (如被大风吹走)

```
现象: 搜索窗口内的气球轮廓迅速变小 → score 可能降低
处理:
    1. 速度预测 (卡尔曼) 会捕捉到运动趋势
    2. 搜索窗口自动跟随
    3. 如果 scale_factor 迅速增加 (说明气球在远离):
       vx 快速增加 (加速追赶)
       → 对数映射天然支持这种情况
```

---

## 17. 性能分析

### 17.1 各阶段每帧耗时估算 (RK3568)

| 阶段 | 操作 | 估算耗时 |
|------|------|----------|
| **SEARCH** (仅 YOLO 帧) | YOLO 预处理+推理+NMS | 25-35ms |
| **SEARCH** (非 YOLO 帧) | 无操作 (丢弃) | <0.1ms |
| **LOCK** | 直方图 + 反向投影 + findContours | 3-8ms |
| **TRACK** (每帧) | 预测+反向投影(局部)+形态学+轮廓打分+minEnclosingCircle | 2-5ms |
| **HOVER** (每帧) | 同 TRACK | 2-5ms |
| **LOST** (每帧) | 同 TRACK (继续尝试) | 2-5ms |

### 17.2 GPU/NPU 占用

```
SEARCH: NPU 负载 100% (YOLO 推理) 持续 ~30ms, 每 2 秒一次
LOCK:   NPU 空闲
TRACK:  NPU 空闲 ← 核心优势: 无需 NPU
HOVER:  NPU 空闲
LOST:   NPU 空闲
```

### 17.3 CPU 占用估算

```
VisionThread (30fps × 3ms average) ≈ 90ms/s → ~9% CPU
其他线程 (MAVLink + 控制) → ~5% CPU
总计 → ~15% CPU (RK3568 Cortex-A55 ×4)
```

---

## 18. 文件清单与接口

### 18.1 新增/修改文件

```
balloon_tracker/
├── include/
│   ├── data/
│   │   ├── vision_detector.h        # VisionDetector 类
│   │   ├── color_tracker.h          # ColorTracker 类 (核心)
│   │   ├── yolo_detector.h          # YoloDetector 类
│   │   └── visual_servo.h           # VisualServo 类 (控制输出)
│   │
│   └── types.h                      # 新增: VisionPhase, TrackingResult 等
│
├── src/
│   └── data/
│       ├── vision_detector.cpp      # 双模调度 (SEARCH→LOCK→TRACK→HOVER→LOST)
│       ├── color_tracker.cpp        # 颜色追踪核心逻辑
│       ├── yolo_detector.cpp        # YOLO 检测 + NMS
│       └── visual_servo.cpp         # 视觉伺服控制输出
```

### 18.2 公共接口

```cpp
// 主入口: VisualTracker (封装双模)
class VisualTracker {
public:
    bool init(const char* model_path, int camera_device);
    void shutdown();

    // 主更新 (每帧调用一次, 由 VisionThread 驱动)
    VisionOutput process_frame();

    // 强制重新搜索 (决策层调用, 如 LAND 后重新起飞)
    void reset();

    // 获取当前阶段
    VisionPhase phase() const;
};
```

### 18.3 与现有架构的集成

```
main.cpp 中的 VisionThread:
{
    VisualTracker tracker;
    tracker.init("model/balloon_int8.rknn", 0);

    while (g_running) {
        VisionOutput output = tracker.process_frame();

        // 根据 output.phase 决定如何处理:
        switch (output.phase) {
        case VisionPhase::SEARCH:
            // 将 YOLO 结果 (如果有) 传给状态机
            break;
        case VisionPhase::TRACK:
        case VisionPhase::HOVER:
            // 将 TrackingResult 中的控制指令直接写入 g_motion_command
            break;
        case VisionPhase::LOST:
            // 告知状态机目标丢失
            break;
        }
    }
}
```

---

## 19. 开发优先级

| 优先级 | 任务 | 依赖 | 说明 |
|--------|------|------|------|
| **P0** | `types.h` 扩展 | 无 | 添加 VisionPhase, TrackingResult 等结构 |
| **P0** | `color_tracker.h/cpp` | types.h | 核心颜色追踪逻辑 |
| **P0** | `visual_servo.h/cpp` | types.h | 视觉伺服控制输出 |
| **P1** | `yolo_detector.h/cpp` | types.h | 封装已有 YOLO 代码 |
| **P1** | `vision_detector.h/cpp` | P0 + P1 全部 | 双模调度器 |
| **P1** | 卡尔曼预测器 | color_tracker | 自适应搜索窗口 |
| **P2** | 递推 A₀ 更新 | color_tracker | 在线优化参考值 |
| **P2** | 预防性降级检测 | vision_detector | 光照变化防御 |
| **P2** | 调试可视化 | 全部 | 叠加 bbox/轮廓/scale 到画面 |
| **P3** | 参数微调 | P0-P2 | 实地飞行调参 |

---

## 20. 调试与可视化

### 20.1 叠加显示

开发阶段在画面上叠加以下信息 (传到 HDMI 或保存为视频):

```
┌─────────────────────────────────────────────┐
│  Phase: TRACK    Score: 0.87     FPS: 28    │
│  Scale: 0.52    Target: 0.50    vx: +0.3   │
│  Lost: 0          A0: 5234      A: 4900    │
│                                             │
│        ┌────────────┐                        │
│        │  搜索窗口   │ ← 绿色矩形             │
│        │  ┌──────┐  │                        │
│        │  │  ●   │  │ ← 蓝色圆 = minEnclosing│
│        │  │气球  │  │                        │
│        │  │轮廓  │  │ ← 红色 = findContours  │
│        │  └──────┘  │                        │
│        └────────────┘                        │
│                                             │
│  [Histogram Preview]                        │
│  ┌────────────────────┐                     │
│  │ ░░░░░░░░           │                     │
│  │ ░░████░░           │ ← H-S 直方图山峰     │
│  │ ░░████░░           │                     │
│  │ ░░░░░░░░           │                     │
│  └────────────────────┘                     │
└─────────────────────────────────────────────┘
```

### 20.2 关键日志

```
[INFO] Vision: phase=SEARCH, yolo_fps=2.0
[INFO] Vision: phase=LOCK, detected_bbox=(320,200,80,90) conf=0.87
[INFO] Vision: color_sampled, h_peak=4, s_peak=185, circularity=0.91
[INFO] Vision: A0=5234, centered(x_err=0.03, y_err=-0.02)
[INFO] Vision: phase=TRACK, fps=28, scale=1.00->0.52, vx=+0.3
[WARN] Vision: tracking_degraded, avg_score=0.52, triggering YOLO recheck
[INFO] Vision: phase=LOST, duration=3.2s, reverting_to_SEARCH
```

### 20.3 数据录制

开发阶段录制以下数据用于离线分析:

```
每帧:
    timestamp, phase, cx, cy, radius, A, A0, scale_factor,
    best_score, circularity, x_error, y_error,
    cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, lost_counter

格式: CSV (一行一帧)
```

---

## 附录 A: 与纯 YOLO 方案对比

| 维度 | 纯 YOLO (低频) | YOLO + 颜色追踪 (本方案) |
|------|---------------|-------------------------|
| 控制延迟 | 0.5s (YOLO 刷新率) | 1/30s (追踪频率) |
| NPU 占用 | 始终 100% | 仅 SEARCH 阶段 100%, 其余 0% |
| 位置精度 | 粗 (矩形 bbox, ±10px) | 精 (像素级轮廓, ±2px) |
| 光照变化 | 不受影响 (特征) | 受影响但自适应采样 + 预防性降级 |
| 误检处理 | NMS filtering | 形状验证 + YOLO 兜底 |
| 计算开销 | NPU: 高, CPU: 低 | NPU: 低, CPU: 中 |
| 适用距离 | 全部 | bbox > 40×40px |
