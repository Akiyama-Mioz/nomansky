/**
 * main.cc — GStreamer 摄像头采集 + YOLO 气球检测 + HDMI 显示
 *
 * 对比 zero3w_yolo_camera_demo/main.cc:
 *   扔掉 100 行 V4L2 multiplanar ioctl,
 *   改用 GStreamer v4l2src → appsink 管道,
 *   不再调用 S_FMT, 不再跟 rkaiq 抢 ISP 配置.
 *
 * 编译: make
 * 运行: LD_LIBRARY_PATH=/usr/local/lib ./zero3w_camera_ai
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <mutex>

#include <linux/fb.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <rknn_api.h>

// ===================== 常量 =====================
static const int   MODEL_W = 320, MODEL_H = 320;
static const float CONF_THR = 0.25f;
static const float NMS_THR  = 0.45f;
static const int   FB_W = 1920, FB_H = 1080;

// ===================== 全局 =====================
static volatile int g_running = 1;
static void on_signal(int) { g_running = 0; }

// ── RKNN ──
static rknn_context g_ctx;
static int g_nbox;

struct Box { float x1, y1, x2, y2, conf; };

// ── HDMI framebuffer ──
static int  fb_fd = -1;

// ===================== HDMI =====================
static int fb_init() {
    fb_fd = open("/dev/fb0", O_RDWR);
    if (fb_fd < 0) { perror("fb"); return -1; }
    struct fb_var_screeninfo vi;
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vi) < 0) { close(fb_fd); return -1; }
    printf("[FB] %dx%d %dbpp\n", vi.xres, vi.yres, vi.bits_per_pixel);
    return 0;
}

static void fb_show(const cv::Mat &img) {
    if (fb_fd < 0) return;
    cv::Mat rsz, bgra;
    if (img.cols != FB_W || img.rows != FB_H)
        cv::resize(img, rsz, cv::Size(FB_W, FB_H));
    else
        rsz = img;
    cv::cvtColor(rsz, bgra, cv::COLOR_BGR2BGRA);

    // 一次 write 全屏 (走 VFS 内核分块，避免 DMA 冲突)
    lseek(fb_fd, 0, SEEK_SET);
    write(fb_fd, bgra.data, FB_W * FB_H * 4);
}

static void fb_close() {
    if (fb_fd >= 0) close(fb_fd);
    fb_fd = -1;
}

// ===================== GStreamer 采集 =====================
static bool gst_pull_frame(cv::Mat &bgr) {
    static GstElement    *pipeline = nullptr;
    static GstAppSink    *sink     = nullptr;

    if (!pipeline) {
        GError *error = nullptr;
        pipeline = gst_parse_launch(
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,format=NV12,width=1920,height=1080 ! "
            "appsink name=sink", &error);
        if (!pipeline) {
            fprintf(stderr, "GStreamer: %s\n", error->message);
            return false;
        }
        sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
        gst_app_sink_set_max_buffers(sink, 2);
        gst_app_sink_set_drop(sink, true);

        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            fprintf(stderr, "GStreamer: failed to start\n");
            return false;
        }
        printf("[GST] v4l2src → NV12 → appsink\n");
    }

    GstSample *sample = gst_app_sink_try_pull_sample(sink, 100 * GST_MSECOND);
    if (!sample) return false;

    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_READ);

    cv::Mat yuv(1080 * 3 / 2, 1920, CV_8UC1, map.data);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);
    return true;
}

// ── Selfpath 采集 (低分辨率, YOLO 专用) ──
static int g_self_w = 0, g_self_h = 0;

static bool gst_pull_selfpath(cv::Mat &bgr) {
    static GstElement *pipeline = nullptr;
    static GstAppSink *sink = nullptr;

    if (!pipeline) {
        GError *error = nullptr;
        pipeline = gst_parse_launch(
            "v4l2src device=/dev/video1 ! "
            "video/x-raw,format=NV12,width=640,height=480 ! "
            "appsink name=sink", &error);
        if (!pipeline) { return false; }
        sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
        gst_app_sink_set_max_buffers(sink, 2);
        gst_app_sink_set_drop(sink, true);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        printf("[GST-self] /dev/video1 640x480 NV12\n");
    }

    GstSample *sample = gst_app_sink_try_pull_sample(sink, 50 * GST_MSECOND);
    if (!sample) return false;

    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_READ);

    if (g_self_w == 0) {
        g_self_h = 480;
        g_self_w = (int)map.size * 2 / 3 / g_self_h;
    }
    cv::Mat yuv(g_self_h * 3 / 2, g_self_w, CV_8UC1, map.data);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);
    return true;
}

// ===================== RKNN =====================
static int rknn_load(const char *path) {
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror("model"); return -1; }
    fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
    uint8_t *d = (uint8_t*)malloc(sz); fread(d, 1, sz, fp); fclose(fp);
    int ret = rknn_init(&g_ctx, d, sz, 0, nullptr); free(d);
    if (ret < 0) { fprintf(stderr, "rknn_init=%d\n", ret); return -1; }

    rknn_tensor_attr out_attr;
    memset(&out_attr, 0, sizeof(out_attr)); out_attr.index = 0;
    rknn_query(g_ctx, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
    g_nbox = out_attr.dims[2];
    printf("[MODEL] 320x320x3 nbox=%d\n", g_nbox);
    return 0;
}

static void rknn_unload() { rknn_destroy(g_ctx); }

static std::vector<Box> rknn_infer(const cv::Mat &img) {
    cv::Mat rsz, rgb;
    cv::resize(img, rsz, cv::Size(MODEL_W, MODEL_H));
    cv::cvtColor(rsz, rgb, cv::COLOR_BGR2RGB);

    rknn_input ri; memset(&ri, 0, sizeof(ri));
    ri.index = 0; ri.type = RKNN_TENSOR_UINT8;
    ri.size = MODEL_W * MODEL_H * 3; ri.buf = rgb.data;
    ri.fmt = RKNN_TENSOR_NHWC;
    rknn_inputs_set(g_ctx, 1, &ri);
    rknn_run(g_ctx, nullptr);

    rknn_output ro; memset(&ro, 0, sizeof(ro));
    ro.index = 0; ro.want_float = 1;
    rknn_outputs_get(g_ctx, 1, &ro, nullptr);
    float *out = (float*)ro.buf;

    std::vector<Box> boxes;
    const int grids[] = {40, 20, 10};
    int head_offset = 0;
    for (int s = 0; s < 3; s++) {
        int grid = grids[s];
        for (int gy = 0; gy < grid; gy++) {
            for (int gx = 0; gx < grid; gx++) {
                int idx = head_offset + gy * grid + gx;
                float cx = out[idx + 0 * g_nbox];
                float cy = out[idx + 1 * g_nbox];
                float w  = out[idx + 2 * g_nbox];
                float h  = out[idx + 3 * g_nbox];
                float cf = out[idx + 4 * g_nbox];
                if (cf < CONF_THR || w <= 0 || h <= 0) continue;
                float scale_x = (float)img.cols / MODEL_W;
                float scale_y = (float)img.rows / MODEL_H;
                float x1 = (cx - w/2) * scale_x;
                float y1 = (cy - h/2) * scale_y;
                float x2 = (cx + w/2) * scale_x;
                float y2 = (cy + h/2) * scale_y;
                boxes.push_back({x1, y1, x2, y2, cf});
            }
        }
        head_offset += grid * grid;
    }
    rknn_outputs_release(g_ctx, 1, &ro);

    // NMS
    std::sort(boxes.begin(), boxes.end(),
              [](const Box &a, const Box &b) { return a.conf > b.conf; });
    std::vector<Box> nms;
    std::vector<bool> sup(boxes.size(), false);
    for (size_t i = 0; i < boxes.size(); i++) {
        if (sup[i]) continue;
        nms.push_back(boxes[i]);
        for (size_t j = i + 1; j < boxes.size(); j++) {
            if (sup[j]) continue;
            float xx1 = std::max(boxes[i].x1, boxes[j].x1);
            float yy1 = std::max(boxes[i].y1, boxes[j].y1);
            float xx2 = std::min(boxes[i].x2, boxes[j].x2);
            float yy2 = std::min(boxes[i].y2, boxes[j].y2);
            float ow = std::max(0.f, xx2 - xx1), oh = std::max(0.f, yy2 - yy1);
            float area_i = (boxes[i].x2 - boxes[i].x1) * (boxes[i].y2 - boxes[i].y1);
            float area_j = (boxes[j].x2 - boxes[j].x1) * (boxes[j].y2 - boxes[j].y1);
            if (ow * oh / (area_i + area_j - ow * oh + 1e-6f) > NMS_THR) sup[j] = true;
        }
    }
    return nms;
}

static void draw_boxes(cv::Mat &img, const std::vector<Box> &bxs) {
    for (auto &b : bxs) {
        cv::rectangle(img, cv::Point(b.x1, b.y1), cv::Point(b.x2, b.y2),
                      cv::Scalar(0, 255, 0), 2);
        char lb[64]; snprintf(lb, sizeof(lb), "balloon %.0f%%", b.conf * 100);
        cv::putText(img, lb, cv::Point(b.x1, b.y1 - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}

// ===================== main =====================
int main(int argc, char **argv) {
    const char *model = (argc >= 2) ? argv[1] : "ballon_int8.rknn";

    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    printf("========================================\n");
    printf("  Zero 3W — GStreamer Camera + Balloon AI\n");
    printf("========================================\n");

    // 1. HDMI
    if (fb_init() != 0) return 1;

    // 2. RKNN
    if (rknn_load(model) != 0) { fb_close(); return 1; }

    // 3. GStreamer init
    gst_init(nullptr, nullptr);

    // 4. Main loop
    int fcnt = 0, dcnt = 0;
    printf("[RUN] Ctrl+C to stop\n");

    while (g_running) {
        cv::Mat frame;
        if (!gst_pull_frame(frame)) {
            if (fcnt == 0) printf("[GST] waiting for first frame...\n");
            usleep(10000);
            continue;
        }
        fcnt++;

        // selfpath 小图给 YOLO (640→320 resize, ~1ms NV12→BGR)
        cv::Mat detect_frame;
        bool has_self = gst_pull_selfpath(detect_frame);
        dcnt++;
        auto boxes = has_self ? rknn_infer(detect_frame) : rknn_infer(frame);

        // 缩放检测框到 1080p 显示坐标系
        cv::Mat out = frame.clone();
        float sx = (float)out.cols / (has_self ? detect_frame.cols : frame.cols);
        float sy = (float)out.rows / (has_self ? detect_frame.rows : frame.rows);
        for (auto &b : boxes) { b.x1 *= sx; b.y1 *= sy; b.x2 *= sx; b.y2 *= sy; }
        draw_boxes(out, boxes);
        fb_show(out);
        printf("[%d] %zu balloons (src=%s)\n", dcnt, boxes.size(),
               has_self ? "self" : "main");
    }

    // 5. Cleanup
    printf("\n[DONE] %d frames, %d inferences\n", fcnt, dcnt);
    // GStreamer pipeline is static inside gst_pull_frame, cleaned up at exit
    rknn_unload();
    fb_close();
    return 0;
}
