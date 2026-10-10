"""移动适配网页和只保留最新帧的、多客户端 HTTP MJPEG 服务。

/video 和 / 是手机页面；/stream.mjpg 是原始 MJPEG；/video?raw=1 兼容原始流。
frame_age_ms 仅表示服务器最新 JPEG 的年龄，不是端到端显示延迟。
"""
import copy
import json
import math
import socket
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

try:
    import cv2
except ImportError:
    cv2 = None

_latest_jpg = None
_stream_seq = 0
_last_frame_at = None
_status = {}
_frames = threading.Condition()


def update_status(metadata):
    """更新状态，不伪造新视频帧或刷新最后一帧的时间。

    推荐字段：frame_id,width,height,fps,backend,motion_mode,visible,predicted,
    confidence,du_px,dv_px,state,stream_connected。额外 JSON 字段原样保留。
    """
    if metadata is not None:
        with _frames:
            _status.update(copy.deepcopy(dict(metadata)))


def push_frame(jpg_bytes, metadata=None):
    """原子发布 JPEG 与对应元数据；慢客户端只取最新帧，不保存历史队列。"""
    global _latest_jpg, _stream_seq, _last_frame_at
    jpg = bytes(jpg_bytes)
    if not jpg:
        raise ValueError("JPEG 不能为空")
    with _frames:
        if metadata is not None:
            _status.update(copy.deepcopy(dict(metadata)))
        _latest_jpg = jpg
        _stream_seq += 1
        _last_frame_at = time.monotonic()
        _frames.notify_all()


def _status_snapshot(server_id):
    with _frames:
        result = copy.deepcopy(_status)
        result.update(server_id=server_id, stream_seq=_stream_seq,
                      has_frame=_latest_jpg is not None,
                      frame_age_ms=(round((time.monotonic() - _last_frame_at) * 1000, 1)
                                    if _last_frame_at is not None else None))
        return result


_PAGE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>智能云台实时画面</title><style>
*{box-sizing:border-box}html,body{margin:0;width:100%;height:100%;background:#080c12;color:#eef3fa;font-family:system-ui,-apple-system,sans-serif}
.app{height:100vh;height:100dvh;display:flex;flex-direction:column;padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
header{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:8px 12px;background:#172130;font-size:14px;flex:none}
button{background:#2d4767;color:white;border:1px solid #54759b;border-radius:6px;padding:5px 8px;font:inherit;font-size:12px}
.status{display:grid;grid-template-columns:1fr;gap:3px;padding:7px 12px;background:#101924;font-size:12px;line-height:1.35;flex:none;overflow-wrap:anywhere}
.stage{flex:1;min-height:0;min-width:0;display:flex;align-items:center;justify-content:center;overflow:hidden;background:#000}
.stage img{display:block;width:100%;height:100%;min-width:0;min-height:0;object-fit:contain;object-position:center}
.warn{color:#ffce79}.ok{color:#8ae9b0}.note{color:#b4c3d5;font-size:11px}
@media (orientation:landscape){header{padding:4px 10px;font-size:12px}.status{grid-template-columns:1fr 1fr;padding:4px 10px;font-size:11px;gap:2px 12px}.note{font-size:10px}}
</style></head><body><main class="app">
<header><strong>智能云台实时画面</strong><div><button id="reconnect">重新连接</button> <button id="fullscreen">全屏</button></div></header>
<section class="status" aria-live="polite">
<div id="videoInfo">视频：等待首帧</div><div id="backendInfo">检测后端：读取中 · 运动模式：读取中</div>
<div id="targetInfo">未发现有效目标</div><div id="freshness" class="warn">画面新鲜度：读取中</div>
<div class="note">完整画面等比显示；新鲜度仅为服务器最新帧年龄，不是端到端延迟。</div>
</section><section class="stage"><img id="picture" alt="实时视频，完整等比显示"></section>
</main><script>
const picture=document.getElementById('picture');let reconnectTimer=null;let lastServer=null;
const number=(v,n=1)=>Number.isFinite(Number(v))?Number(v).toFixed(n):'--';
function connect(){clearTimeout(reconnectTimer);picture.src='/stream.mjpg?client='+Date.now();}
document.getElementById('reconnect').onclick=connect;
document.getElementById('fullscreen').onclick=()=>{const fn=document.documentElement.requestFullscreen;if(fn)fn.call(document.documentElement).catch(()=>{});};
picture.onerror=()=>{if(!document.hidden)reconnectTimer=setTimeout(connect,1500);};
document.addEventListener('visibilitychange',()=>{if(document.hidden){clearTimeout(reconnectTimer);picture.removeAttribute('src');}else connect();});
async function poll(){
 if(!document.hidden){try{
  const response=await fetch('/status.json',{cache:'no-store'});if(!response.ok)throw new Error('HTTP '+response.status);
  const s=await response.json();if(lastServer&&s.server_id!==lastServer)connect();lastServer=s.server_id;
  document.getElementById('videoInfo').textContent='视频：'+(s.width&&s.height?s.width+'×'+s.height:'等待首帧')+' · '+number(s.fps)+' FPS · 帧 #'+(s.frame_id??'--');
  document.getElementById('backendInfo').textContent='检测后端：'+(s.backend??'--')+' · 运动模式：'+(s.motion_mode??'--');
  const visible=Boolean(s.visible)&&!s.predicted;
  document.getElementById('targetInfo').textContent=visible?'有效目标'+(s.label?' '+s.label:'')+' · 置信 '+number(s.confidence??s.conf,2)+' · du '+number(s.du_px)+' / dv '+number(s.dv_px)+' px':'未发现有效目标';
  const fresh=document.getElementById('freshness');const age=s.frame_age_ms;
  const ok=s.has_frame&&age!==null&&age<2000&&s.stream_connected!==false;
  fresh.className=ok?'ok':'warn';fresh.textContent='画面新鲜度：'+(!s.has_frame?'等待首帧':s.stream_connected===false?'视频断流，等待重连 · '+number(age,0)+' ms':number(age,0)+' ms'+(ok?'':'（最新帧已陈旧）'));
 }catch(e){const fresh=document.getElementById('freshness');fresh.className='warn';fresh.textContent='状态连接失败：'+e.message;}}
 setTimeout(poll,500);
}
connect();poll();
</script></body></html>'''.encode('utf-8')


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(2.0)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _headers(self, code, content_type, content_length=None, stream=False):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("X-Content-Type-Options", "nosniff")
        if content_length is not None:
            self.send_header("Content-Length", str(content_length))
        if stream:
            self.send_header("Connection", "close")
            self.close_connection = True
        self.end_headers()

    def do_GET(self):
        request = urlsplit(self.path)
        raw = request.path == "/video" and "1" in parse_qs(request.query).get("raw", [])
        try:
            if request.path == "/stream.mjpg" or raw:
                self._stream()
            elif request.path in ("/", "/video"):
                self._headers(200, "text/html; charset=utf-8", len(_PAGE))
                self.wfile.write(_PAGE)
            elif request.path == "/status.json":
                body = json.dumps(_status_snapshot(self.server.server_id), ensure_ascii=False).encode('utf-8')
                self._headers(200, "application/json; charset=utf-8", len(body))
                self.wfile.write(body)
            else:
                self._headers(404, "text/plain; charset=utf-8", 0)
        except OSError:
            self.close_connection = True  # 断开/慢客户端不拖住主循环或其他客户端

    def _stream(self):
        self._headers(200, "multipart/x-mixed-replace; boundary=frame", stream=True)
        sent_seq = 0  # 新客户端立即从全局最新帧开始，不回放历史
        while not self.server.stop_event.is_set():
            with _frames:
                _frames.wait_for(lambda: self.server.stop_event.is_set() or
                                 (_latest_jpg is not None and _stream_seq > sent_seq), timeout=2.0)
                if self.server.stop_event.is_set():
                    return
                if _latest_jpg is None or _stream_seq <= sent_seq:
                    continue
                jpg, seq = _latest_jpg, _stream_seq
            header = ("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n"
                      "X-Stream-Seq: %d\r\n\r\n" % (len(jpg), seq)).encode('ascii')
            self.wfile.write(header + jpg + b"\r\n")
            self.wfile.flush()
            sent_seq = seq

    def log_message(self, *args):
        pass


class _MJPEGServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, *args, **kwargs):
        self.stop_event = threading.Event()
        self.server_id = uuid.uuid4().hex
        super().__init__(*args, **kwargs)

    def _stop_streams(self):
        self.stop_event.set()
        with _frames:
            _frames.notify_all()

    def shutdown(self):
        self._stop_streams()
        super().shutdown()

    def server_close(self):
        self._stop_streams()
        super().server_close()


def draw_overlay(img, det, state, fps, rng, mode, focus_dac=None,
                 frame_id=0, backend="mock"):
    """绘制紧凑 HUD；仅裁剪绘制坐标，绝不修改 raw bbox 或控制 du/dv。"""
    h, w = img.shape[:2]
    if det.visible and not det.predicted:
        raw = tuple(float(v) for v in det.bbox_xyxy)
        if (len(raw) == 4 and all(math.isfinite(v) for v in raw) and
                raw[2] > raw[0] and raw[3] > raw[1] and
                raw[0] < w and raw[1] < h and raw[2] >= 0 and raw[3] >= 0):
            x1, y1 = max(0, min(w - 1, int(raw[0]))), max(0, min(h - 1, int(raw[1])))
            x2, y2 = max(0, min(w - 1, int(raw[2]))), max(0, min(h - 1, int(raw[3])))
            color = (0, 190, 255) if getattr(det, "observation_source", "YOLO") == "FEAR" else (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label = f"{det.label} {det.conf:.2f}"
            label_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0]
            label_x = max(3, min(x1, w - label_width - 3))
            label_y = max(16, min(h - 3, y1 - 5))
            cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)
    cv2.drawMarker(img, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 18, 1)
    lines = [f"#{frame_id}  {w}x{h}  {fps:.1f} FPS",
             f"DET {str(backend)[:32]} | MOT {mode} | {state}"]
    font = cv2.FONT_HERSHEY_SIMPLEX
    widest = max(cv2.getTextSize(text, font, 0.43, 1)[0][0] for text in lines)
    scale = 0.43 * min(1.0, max(0.2, (w - 12) / max(1, widest)))
    line_h = max(12, cv2.getTextSize("Ag", font, scale, 1)[0][1] + 7)
    hud_h = min(h, line_h * 2 + 5)
    shade = img[:hud_h, :].copy()
    shade[:] = 0
    img[:hud_h, :] = cv2.addWeighted(shade, 0.65, img[:hud_h, :], 0.35, 0)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (5, (i + 1) * line_h), font, scale,
                    (240, 245, 250), 1, cv2.LINE_AA)
    return img


def serve(port, try_range=10):
    """保留 (server, actual_port) 返回；端口被占用时自动顺延。"""
    if cv2 is None:
        raise RuntimeError("没装 opencv")
    last_err = None
    for p in range(port, port + try_range):
        try:
            srv = _MJPEGServer(("0.0.0.0", p), _Handler)
            actual_port = srv.server_address[1]
            print(f"[画面] 浏览器打开 http://<jetson-ip>:{actual_port}/video（手机适配）")
            if p != port:
                print(f"[画面] 注意：{port} 被占用，已顺延到 {actual_port}；"
                      f"查谁占着：sudo lsof -i :{port}")
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            return srv, actual_port
        except OSError as e:
            last_err = e
    raise RuntimeError(f"端口 {port}-{port + try_range - 1} 全被占用：{last_err}")
