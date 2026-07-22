#!/usr/bin/env python3
"""
PX4 MAVLink HDMI HUD Overlay
==============================
Compact MSI Afterburner-style overlay in top-left corner.
Rest of screen reserved for camera feed.

Usage:
  from hud_renderer import HUD
  hud = HUD()
  hud.open()
  hud.update(telemetry, cmd_state)
"""

import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np


# ============================================================
# Colors (BGR)
# ============================================================
C_BG       = (0, 0, 0)         # transparent / camera bg
C_PANEL    = (30, 30, 36)      # panel bg (dark gray, not pure black)
C_BORDER   = (55, 55, 60)      # border
C_TEXT     = (200, 200, 200)   # normal text
C_LABEL    = (120, 120, 130)   # dim labels
C_VALUE    = (235, 235, 235)   # bright values
C_TITLE    = (90, 170, 250)    # section title blue
C_OK       = (70, 210, 70)     # green
C_WARN     = (70, 190, 250)    # orange
C_FAIL     = (70, 70, 250)     # red
C_BAR_BG   = (38, 38, 42)      # bar background
C_BAR_OK   = (70, 180, 70)     # bar green
C_BAR_WARN = (70, 180, 250)    # bar orange

FONT = cv2.FONT_HERSHEY_SIMPLEX


class HUD:
    """Compact overlay HUD — MSI Afterburner style, top-left corner."""

    def __init__(self, fb_device="/dev/fb0", width=1920, height=1080):
        self.fb_device = fb_device
        self.width = width
        self.height = height
        self._fb_fd = None
        self._fb = None
        self._fb_size = 0
        self._fb_fmt = "BGRA"
        self._fb_bytes = 4
        self._canvas = np.zeros((height, width, 3), dtype=np.uint8)
        self._bgra_buf = np.zeros((height, width, 4), dtype=np.uint8)
        self._log_lines = deque(maxlen=5)
        self._running = True
        self._start_time = time.time()

        # ── Vertical panel geometry (top-left) ──
        self._panel_x = 16
        self._panel_y = 16
        self._panel_w = 256
        self._pad_x = 8
        self._label_w = 72

    # ── Lifecycle ─────────────────────────────────────────

    def open(self):
        try:
            self._fb_fd = os.open(self.fb_device, os.O_RDWR)
        except OSError as e:
            print("[HUD] Cannot open {}: {}".format(self.fb_device, e))
            self._fb_fd = None
            return False

        import fcntl, struct, mmap
        FBIOGET_VSCREENINFO = 0x4600
        FBIOGET_FSCREENINFO = 0x4602

        var_buf = bytearray(160)
        try:
            fcntl.ioctl(self._fb_fd, FBIOGET_VSCREENINFO, var_buf)
            fields = struct.unpack("IIIIIIIIIIIIIIIIIIII", bytes(var_buf[:80]))
            self.width = fields[0]
            self.height = fields[1]
            fb_vw = fields[2]
            fb_vh = fields[3]
            fb_bpp = fields[6]
            r_off = fields[8]
            b_off = fields[14]
            print("[HUD] {}x{} {}bpp R@{} B@{}".format(
                self.width, self.height, fb_bpp, r_off, b_off))

            # Detect pixel format
            if fb_bpp == 32:
                self._fb_fmt = "BGRA" if b_off == 0 else "RGBA"
                self._fb_bytes = 4
            elif fb_bpp == 16:
                self._fb_fmt = "RGB565" if b_off == 0 else "BGR565"
                self._fb_bytes = 2
            else:
                self._fb_fmt = "BGRA"; self._fb_bytes = 4
        except Exception:
            fb_vw = self.width; fb_vh = self.height

        self._canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._bgra_buf = np.zeros((self.height, self.width, 4), dtype=np.uint8)

        stride = fb_vw * self._fb_bytes
        self._fb_size = stride * fb_vh
        fix_buf = bytearray(80)
        try:
            fcntl.ioctl(self._fb_fd, FBIOGET_FSCREENINFO, fix_buf)
            smem_len = struct.unpack("16sIIIIIIIHHI48s", bytes(fix_buf[:80]))[1]
            if smem_len > 0: self._fb_size = smem_len
        except Exception:
            pass

        try:
            self._fb = mmap.mmap(self._fb_fd, self._fb_size,
                                 mmap.MAP_SHARED, mmap.PROT_WRITE)
        except Exception as e:
            print("[HUD] mmap failed: {}".format(e))
            self._fb = None

        self._log("HUD ready {}x{} {}".format(self.width, self.height, self._fb_fmt))
        return True

    def close(self):
        self._running = False
        if self._fb: self._fb.close()
        if self._fb_fd: os.close(self._fb_fd)

    def _log(self, text):
        self._log_lines.append("[{}] {}".format(time.strftime("%H:%M:%S"), text))

    # ── Main update ──────────────────────────────────────

    def update(self, telemetry, cmd_state=None):
        if cmd_state is None:
            cmd_state = {}

        self._canvas[:] = C_BG

        x0 = self._panel_x
        y0 = self._panel_y
        px = self._pad_x
        pw = self._panel_w

        # ── Gather data ──
        conn_ok = telemetry.get("connected", False)
        mode = telemetry.get("mode_name", "--")
        armed = "ARM" if telemetry.get("armed") else "DSRM"
        state = telemetry.get("state_name", "--")
        roll  = telemetry.get("roll_deg", 0) or 0
        pitch = telemetry.get("pitch_deg", 0) or 0
        yaw   = telemetry.get("yaw_deg", 0) or 0
        alt = telemetry.get("alt_rel")
        alt_s = "{:.1f} m".format(alt) if alt is not None else "--"
        vx = telemetry.get("vx") or 0; vy = telemetry.get("vy") or 0; vz = telemetry.get("vz") or 0
        bat = telemetry.get("battery_v")
        bat_s = "{:.1f} V".format(bat) if bat is not None and bat < 100 else "--"
        ekf = "OK" if telemetry.get("ekf_ok") else "OFF"

        last_cmd = cmd_state.get("last_cmd", "-")
        last_res = cmd_state.get("last_result", "-")
        vel = cmd_state.get("current_vel", {})
        vx_c = vel.get("vx", 0) or 0; vy_c = vel.get("vy", 0) or 0
        vz_c = vel.get("vz", 0) or 0; yr_c = vel.get("yaw_rate", 0) or 0
        history = cmd_state.get("history", [])
        log_lines = list(self._log_lines)[-5:]

        # ── Build vertical rows ──
        rows = []
        def add(title, value, color=C_VALUE):
            rows.append((title, str(value), color))

        def add_sep(title):
            rows.append(("---", title, C_TITLE))

        add_sep("TELEMETRY")
        add("Link",    "OK" if conn_ok else "NO",
            C_OK if conn_ok else C_FAIL)
        add("Mode",    mode, C_VALUE)
        add("Armed",   armed, C_OK if telemetry.get("armed") else C_FAIL)
        add("State",   state, C_LABEL)
        add("Roll",    "{:+.1f}°".format(roll), C_VALUE)
        add("Pitch",   "{:+.1f}°".format(pitch), C_VALUE)
        add("Yaw",     "{:.1f}°".format(yaw), C_VALUE)
        add("Alt",     alt_s, C_VALUE)
        add("Vel NED", "{:+.1f} {:+.1f} {:+.1f} m/s".format(vx, vy, vz), C_TEXT)
        add("Bat",     bat_s, C_VALUE)
        add("EKF",     ekf, C_OK if telemetry.get("ekf_ok") else C_FAIL)

        add_sep("COMMANDS")
        add("Last Cmd",  last_cmd, C_TEXT)
        add("Result",    last_res,
            C_OK if last_res in ("ACCEPTED","OK","DONE","SENT")
            else C_WARN if "REJECT" in str(last_res) else C_FAIL)
        add("Vel Cmd",   "{:+.1f} {:+.1f} {:+.1f} yr={:.2f}".format(vx_c, vy_c, vz_c, yr_c),
            C_OK if (abs(vx_c)+abs(vy_c)+abs(vz_c))>0.001 else C_LABEL)
        for h in history[-3:]:
            add("  {}".format(h.get("time","")),
                "{} -> {}".format(h.get("cmd",""), h.get("result","")), C_LABEL)

        add_sep("LOG")
        for line in log_lines:
            add("", line, C_LABEL)

        # frame counter so user can see it updating
        self._frame_n = getattr(self, '_frame_n', 0) + 1
        elapsed = time.time() - self._start_time
        add("Frame", "#{}".format(self._frame_n), C_OK)
        add("Time", "{}s".format(int(elapsed)), C_VALUE)

        # ── Calculate panel size ──
        row_h = 22
        panel_h = 20 + len(rows) * row_h + 12

        # ── Draw background ──
        cv2.rectangle(self._canvas, (x0, y0), (x0+pw, y0+panel_h), C_PANEL, -1)
        cv2.rectangle(self._canvas, (x0, y0), (x0+pw, y0+panel_h), C_BORDER, 1)

        # ── Draw rows ──
        for i, (label, value, color) in enumerate(rows):
            y = y0 + 16 + i * row_h
            if label == "---":
                self._put_text(value, x0+px, y+17, FONT, 0.52, C_TITLE, 2)
            elif label:
                self._put_text(label, x0+px, y+17, FONT, 0.45, C_LABEL, 1)
                self._put_text(value, x0+px+self._label_w, y+17, FONT, 0.45, color, 1)
            else:
                self._put_text(value, x0+px, y+17, FONT, 0.36, C_LABEL, 1)

        self._panel_h = panel_h
        self._flip()

    # ── Drawing helpers ──────────────────────────────────

    # ── Drawing helpers ──────────────────────────────────

    def _section_title(self, y, text):
        self._put_text(text, self._panel_x + self._pad_x, y + 18,
                       FONT, 0.55, C_TITLE, 2)

    def _row2(self, y, k1, v1, c1, k2=None, v2=None, c2=None):
        """One row: two key:value pairs."""
        x = self._panel_x + self._pad_x + 4
        # col 1
        self._put_text(k1, x, y + 17, FONT, 0.48, C_LABEL, 1)
        self._put_text(str(v1), x + 54, y + 17, FONT, 0.48, c1, 1)
        # col 2
        if k2:
            x2 = x + 280
            self._put_text(k2, x2, y + 17, FONT, 0.48, C_LABEL, 1)
            self._put_text(str(v2), x2 + 50, y + 17, FONT, 0.48, c2, 1)

    def _att_row(self, y, roll, pitch, yaw):
        """Attitude: label + 2 mini-bars + yaw value."""
        x = self._panel_x + self._pad_x + 4
        self._put_text("Att", x, y + 17, FONT, 0.48, C_LABEL, 1)

        bw = self._bar_w; bh = self._bar_h
        bx = x + 54
        self._mini_bar(bx, y + 5, bw, bh, roll / 45.0)
        self._put_text("R{:+.0f}".format(roll), bx + bw + 6, y + 17,
                       FONT, 0.42, C_VALUE, 1)

        bx2 = bx + bw + 65
        self._mini_bar(bx2, y + 5, bw, bh, pitch / 45.0)
        self._put_text("P{:+.0f}".format(pitch), bx2 + bw + 6, y + 17,
                       FONT, 0.42, C_VALUE, 1)

        self._put_text("Y{:.0f}".format(yaw), x + 470, y + 17,
                       FONT, 0.48, C_VALUE, 1)

    def _mini_bar(self, x, y, w, h, val):
        """Mini bar, val in [-1, 1], 0 = center."""
        val = max(-1.0, min(1.0, val))
        mid = x + w // 2
        cv2.rectangle(self._canvas, (x, y), (x + w, y + h), C_BAR_BG, -1)
        cv2.line(self._canvas, (mid, y), (mid, y + h), C_BORDER, 1)
        if val >= 0:
            bw = int((w // 2) * val)
            cv2.rectangle(self._canvas, (mid, y + 2), (mid + bw, y + h - 2), C_BAR_OK, -1)
        else:
            bw = int((w // 2) * (-val))
            cv2.rectangle(self._canvas, (mid - bw, y + 2), (mid, y + h - 2), C_BAR_WARN, -1)

    def _put_text(self, text, x, y, font, scale, color, thickness=1):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return
        cv2.putText(self._canvas, text, (x, y), font, scale,
                    color, thickness, cv2.LINE_AA)

    # ── Framebuffer output ───────────────────────────────

    def _flip(self):
        if self._fb_fd is None:
            return
        try:
            # 首帧: 全屏清黑 (消除旧画面残留), 分块写避免大内存分配
            if not getattr(self, '_fb_cleared', False):
                chunk = b'\x00' * (self.width * 4 * 64)  # 64 行一块
                os.lseek(self._fb_fd, 0, os.SEEK_SET)
                for _ in range(self.height // 64):
                    os.write(self._fb_fd, chunk)
                self._fb_cleared = True

            x0, y0 = self._panel_x, self._panel_y
            pw = self._panel_w
            ph = getattr(self, '_panel_h', 400)

            # 只取面板区域
            roi = self._canvas[y0:y0+ph, x0:x0+pw]
            bgra = cv2.cvtColor(roi, cv2.COLOR_BGR2BGRA)

            # 逐行 seek+write 到 fb (fb 每行 stride = self.width * 4)
            fb_stride = self.width * 4
            row_bytes = pw * 4
            buf = bgra.tobytes()  # 面板全部行, 连续内存
            for row in range(ph):
                offset = ((y0 + row) * self.width + x0) * 4
                start = row * row_bytes
                end = start + row_bytes
                os.lseek(self._fb_fd, offset, os.SEEK_SET)
                os.write(self._fb_fd, buf[start:end])
        except Exception:
            pass

    @staticmethod
    def _bgr_to_rgb565(bgr):
        h, w = bgr.shape[:2]
        b = bgr[:,:,0].astype(np.uint16)
        g = bgr[:,:,1].astype(np.uint16)
        r = bgr[:,:,2].astype(np.uint16)
        rgb565 = ((r >> 3) & 0x1F) << 11 | ((g >> 2) & 0x3F) << 5 | ((b >> 3) & 0x1F)
        return rgb565.astype(np.uint16).tobytes()

    @staticmethod
    def _read_cpu_temp():
        for p in ["/sys/class/thermal/thermal_zone0/temp",
                   "/sys/class/hwmon/hwmon0/temp1_input"]:
            try:
                with open(p) as f:
                    v = int(f.read().strip())
                    return v / 1000.0 if v > 1000 else v
            except Exception:
                continue
        return None


# ============================================================
# Standalone test
# ============================================================

def main():
    import signal, sys
    try:
        from pymavlink import mavutil
    except ImportError:
        print("[FATAL] pymavlink not installed")
        sys.exit(1)

    PORT = "/dev/ttyS3"
    BAUD = 115200

    print("PX4 HUD Overlay")
    hud = HUD()
    if not hud.open():
        print("[WARN] No framebuffer, console-only mode")

    mav = mavutil.mavlink_connection(PORT, BAUD, source_system=255, source_component=1)
    msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=15)
    if msg is None:
        print("[FATAL] No heartbeat")
        sys.exit(1)
    hud._log("PX4 connected")
    hud._log("TX: Hello, PX4!")
    mav.mav.statustext_send(6, b"Hello, PX4!")

    def shutdown(sig, frame):
        hud._running = False
    signal.signal(signal.SIGINT, shutdown)

    tele = {"connected": True}
    cmd_state = {"last_cmd": "-", "last_result": "-",
                 "current_vel": {"vx":0,"vy":0,"vz":0,"yaw_rate":0}, "history": []}

    mav.port.timeout = 0

    print("Running... Ctrl+C to stop")

    # ═══════════════════════════════════════════════════
    # 线程 A: 串口读 + MAVLink 解析 (只更新 tele, 不碰 fb)
    # ═══════════════════════════════════════════════════
    def serial_loop():
        parser = mav.mav
        msg_count = {}
        last_log = time.time()
        last_mode = None
        last_armed = None
        while hud._running:
            try:
                w = mav.port.in_waiting
            except Exception:
                w = 0
            if w > 0:
                data = mav.port.read(min(w, 2048))
                msgs = parser.parse_buffer(data)
                if msgs:
                    for msg in msgs:
                        t = msg.get_type()
                        msg_count[t] = msg_count.get(t, 0) + 1
                        t = msg.get_type()
                        if t == "HEARTBEAT":
                            tele["armed"] = (msg.base_mode & 128) != 0
                            tele["mode_raw"] = msg.custom_mode
                            tele["mode_name"] = _mode_name(msg.custom_mode)
                            tele["state"] = msg.system_status
                            tele["state_name"] = _state_name(msg.system_status)
                        elif t == "ATTITUDE":
                            tele["roll_deg"] = math.degrees(msg.roll)
                            tele["pitch_deg"] = math.degrees(msg.pitch)
                            tele["yaw_deg"] = math.degrees(msg.yaw)
                        elif t == "LOCAL_POSITION_NED":
                            tele["vx"] = msg.vx; tele["vy"] = msg.vy; tele["vz"] = msg.vz
                        elif t == "GLOBAL_POSITION_INT":
                            tele["alt_rel"] = msg.relative_alt / 1000.0
                        elif t == "BATTERY_STATUS":
                            if msg.voltages:
                                v = msg.voltages[0] / 1000.0
                                if v < 100: tele["battery_v"] = v
                        elif t == "ESTIMATOR_STATUS":
                            flags = getattr(msg, "health_flags", None) or getattr(msg, "flags", 0)
                            tele["ekf_ok"] = (flags & 0x01) != 0 if flags else False
                        elif t == "STATUSTEXT":
                            sev = {0:"E",1:"A",2:"C",3:"ERR",4:"WARN",5:"N",6:"INFO"}
                            hud._log("[{}] {}".format(
                                sev.get(msg.severity,"?"), msg.text[:50]))
            else:
                time.sleep(0.002)

            # 模式/解锁状态变化 → 记日志
            cur_mode = tele.get("mode_name")
            cur_armed = tele.get("armed")
            if cur_mode and cur_mode != last_mode:
                hud._log("Mode: {}".format(cur_mode))
                last_mode = cur_mode
            if cur_armed is not None and cur_armed != last_armed:
                hud._log("Armed: {}".format("YES" if cur_armed else "NO"))
                last_armed = cur_armed

            # 每 5 秒统计一次消息量
            now = time.time()
            if now - last_log > 5 and msg_count:
                items = sorted(msg_count.items(), key=lambda x: -x[1])[:3]
                hud._log("RX: {}".format(" ".join(
                    "{}x{}".format(t, c) for t, c in items)))
                msg_count = {}
                last_log = now

    # ═══════════════════════════════════════════════════
    # 线程 B: HUD 渲染 + 写 fb (不碰串口)
    # ═══════════════════════════════════════════════════
    def hud_loop():
        tick = 1.0 / 10
        next_tick = time.time()
        while hud._running:
            now = time.time()
            if now >= next_tick:
                tele["connected"] = True
                hud.update(tele, cmd_state)
                next_tick = now + tick
            else:
                time.sleep(0.01)

    t_serial = threading.Thread(target=serial_loop, daemon=True)
    t_hud    = threading.Thread(target=hud_loop, daemon=True)
    t_serial.start()
    t_hud.start()

    # 主线程等待 (Ctrl+C 设 hud._running=False)
    while hud._running:
        time.sleep(0.5)

    hud.close()
    print("Done.")


def _mode_name(cm):
    main = (cm >> 16) & 0xFF; sub = (cm >> 24) & 0xFF
    names = {0:"MAN",1:"ALT",2:"POS",3:"AUTO",4:"AUTO",5:"LOIT",6:"OFFB",7:"STAB",8:"RATT"}
    # PX4 AUTO sub-modes (按固件版本可能略有差异)
    subs = {0:"MISSION", 1:"READY", 2:"HOLD", 3:"LOITER",
            4:"LOITER", 5:"LOITER", 8:"RTL", 20:"TKOFF", 21:"LAND", 22:"FOLLOW"}
    n = names.get(main, "M{}".format(main))
    if main in (3, 4):
        return "{}.{}".format(n, subs.get(sub, "SUB{}".format(sub)))
    return n

def _state_name(s):
    return {0:"UNINIT",1:"BOOT",2:"CAL",3:"STBY",4:"ACT",5:"CRIT",6:"EMERG",
            7:"PWROFF",8:"TERM"}.get(s,"?{}".format(s))


if __name__ == "__main__":
    main()
