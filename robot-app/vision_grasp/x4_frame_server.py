#!/usr/bin/env python3
"""X4 取帧服务（Windows 侧，S1）。

在 Windows 上打开 Insta360 X4（UVC「USB 摄像头」模式），把最新帧经 HTTP 送出，
供 WSL 侧的 x4_sdk_bridge.py 取帧。仅做信息传输，不含任何机械臂指令。

用法（Windows，需已 pip install opencv-python）：
    python x4_frame_server.py                 # 设备 0，2880x1440，端口 8200
    python x4_frame_server.py --list          # 列出可用相机（dshow）
    python x4_frame_server.py --index 1 --width 1920 --height 960
    python x4_frame_server.py --port 8200

接口：
    GET /frame.jpg  最新帧 JPEG；无帧时返回 503（不返回旧帧）
    GET /status     JSON：分辨率/累计帧数/掉帧计数/最近取帧时间
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

# ---------------- 采集线程：只保留最新帧 ----------------

class FrameSource:
    def __init__(self, index=0, width=2880, height=1440):
        self.index = index
        self.width = width
        self.height = height
        self.lock = threading.Lock()
        self.latest = None           # (jpeg_bytes, timestamp)
        self.frames = 0
        self.dropped = 0             # 采集失败计数
        self.error = None
        self.running = False

    def open(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"无法打开相机 index={self.index}（用 --list 查看设备列表）")
        # 丢弃自动曝光稳定前的帧
        for _ in range(10):
            cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[frame-server] 相机已打开 index={self.index}，实际分辨率 {w}x{h}")
        return cap

    def loop(self):
        self.running = True
        cap = None
        try:
            cap = self.open()
        except RuntimeError as e:
            self.error = str(e)
            print(f"[frame-server] {e}")
            return
        while self.running:
            ok, frame = cap.read()
            if not ok:
                self.dropped += 1
                time.sleep(0.2)
                continue
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                self.dropped += 1
                continue
            with self.lock:
                self.latest = (buf.tobytes(), time.time())
                self.frames += 1
        cap.release()
        print("[frame-server] 相机已释放")


def make_handler(src: FrameSource):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/frame.jpg":
                with src.lock:
                    item = src.latest
                if item is None:
                    self.send_response(503)
                    msg = b'{"error": "no frame yet"}'
                else:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    msg = item[0]
                self.send_header("Content-Length", str(len(msg)))
                self.end_headers()
                self.wfile.write(msg)
            elif self.path == "/status":
                with src.lock:
                    item = src.latest
                payload = {
                    "ok": src.error is None and item is not None,
                    "error": src.error,
                    "frames": src.frames,
                    "dropped": src.dropped,
                    "latest_age_s": None if item is None else round(time.time() - item[1], 2),
                }
                msg = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(msg)))
                self.end_headers()
                self.wfile.write(msg)
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, fmt, *args):
            pass  # 静默访问日志，避免刷屏

    return Handler


def list_cameras():
    """枚举 dshow 设备名（依赖 opencv-contrib 的 CAP_DSHOW SHOW）。
    不可用时逐个 index 试开。"""
    try:
        indexs = list(range(4))
        for i in indexs:
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                ok, _ = cap.read()
                print(f"  index={i}: {'可用' if ok else '打开但读不到帧'}")
            cap.release()
    except Exception as e:
        print(f"枚举失败: {e}")


def main():
    ap = argparse.ArgumentParser(description="X4 UVC 取帧 HTTP 服务")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--width", type=int, default=2880)
    ap.add_argument("--height", type=int, default=1440)
    ap.add_argument("--port", type=int, default=8200)
    ap.add_argument("--list", action="store_true", help="列出可用相机后退出")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return

    src = FrameSource(args.index, args.width, args.height)
    t = threading.Thread(target=src.loop, daemon=True)
    t.start()
    time.sleep(1.5)  # 等首帧
    with src.lock:
        if src.error or src.latest is None:
            print("[frame-server] 警告：启动时未取到帧，服务仍会启动（相机恢复后自动出帧）")

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(src))
    print(f"[frame-server] http://<本机IP>:{args.port}/frame.jpg  (Ctrl+C 退出)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        src.running = False
        print("\n[frame-server] 已停止")


if __name__ == "__main__":
    main()
