"""HTTP MJPEG 画面服务（文档 §3/§7）。
手机和电脑局域网打开 http://<jetson-ip>:端口/video 看带框实时画面。
叠加：目标框、类别、置信度、du/dv、状态、FPS、距离、mock/real 标识。
客户端慢/断开不拖慢主循环（只取最新一帧）。
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

try:
    import cv2
except ImportError:
    cv2 = None

_latest_jpg = None
_lock = threading.Lock()


def push_frame(jpg_bytes):
    global _latest_jpg
    with _lock:
        _latest_jpg = jpg_bytes


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/video":
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    jpg = _latest_jpg
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass  # 客户端断开，主循环不受影响

    def log_message(self, *a): pass


def draw_overlay(img, det, state, fps, rng, mode):
    """在原图上叠加信息；网页缩放不改变控制坐标（控制只用原图 du/dv）。"""
    x1, y1, x2, y2 = [int(v) for v in det.bbox_xyxy]
    if det.visible and not det.predicted:
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"{det.label} {det.conf:.2f}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    h, w = img.shape[:2]
    cv2.drawMarker(img, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    dist_txt = f"{rng.distance_m:.1f}m" if rng.valid else "N/A"
    lines = [f"[{state}] {mode}  FPS {fps:.1f}",
             f"du {det.du_px:+.0f} dv {det.dv_px:+.0f}  conf {det.conf:.2f}",
             f"dist {dist_txt}"]
    for i, t in enumerate(lines):
        cv2.putText(img, t, (10, 30 + i * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return img


def serve(port, try_range=10):
    """端口被占时自动顺延（8080->8081…），并打印实际地址和查占用命令。"""
    if cv2 is None:
        raise RuntimeError("没装 opencv")
    last_err = None
    for p in range(port, port + try_range):
        try:
            srv = HTTPServer(("0.0.0.0", p), _Handler)
            print(f"[画面] 浏览器打开 http://<jetson-ip>:{p}/video")
            if p != port:
                print(f"[画面] 注意：{port} 被占用，已顺延到 {p}；"
                      f"查谁占着：sudo lsof -i :{port}")
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            return srv, p
        except OSError as e:
            last_err = e
    raise RuntimeError(f"端口 {port}-{port + try_range - 1} 全被占用：{last_err}")
