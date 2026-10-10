"""视频源：RTSP 只保留最新解码帧；断流/过期返回 None，mock 不碰设备。"""
import threading
import time

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class Frame:
    def __init__(self, frame_id, image, ts):
        self.frame_id = frame_id
        self.image = image
        self.ts = ts  # 单调时钟


class RTSPSource:
    def __init__(self, url, reconnect_s=2.0, width=0, height=0):
        self.url, self.reconnect_s = url, reconnect_s
        self.w, self.h = width, height
        self._fid, self._delivered_fid, self._dropped = 0, 0, 0
        self._latest, self._last_frame_ts = None, None
        self._connected, self._last_error = False, None
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._worker = None
        self._first_read = True
        self._max_age_s = 1.0

    def open(self):
        if not HAS_CV2:
            raise RuntimeError("没装 opencv，Jetson 上 pip install opencv-python")
        # smart_gimbal 只走 RTSP 只读流，不直接碰摄像头设备
        # （/dev/video0 由 Codex 的独立运行器占用发流，避免抢设备）
        if self._worker is not None and self._worker.is_alive():
            if self._stop.is_set():
                raise RuntimeError("RTSP 解码线程仍在关闭，不能启动第二个连接")
            return
        self._stop.clear()
        with self._cv:
            self._latest, self._last_frame_ts = None, None
            self._connected, self._first_read = False, True
        self._worker = threading.Thread(target=self._decode_loop,
                                        name="rtsp-latest-frame", daemon=True)
        self._worker.start()

    def _discard_latest_locked(self):
        if self._latest is not None and self._latest.frame_id > self._delivered_fid:
            self._dropped += 1
        self._latest = None

    def _decode_loop(self):
        # 连接、读取、释放始终由同一线程拥有，close 不跨线程 release。
        while not self._stop.is_set():
            cap = None
            try:
                # ffmpeg 打开/读取超时仍为 5 秒；不增加其他相机或流连接。
                cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, [
                    cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                    cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000,
                ])
                if self._stop.is_set():
                    break
                if not cap.isOpened():
                    raise RuntimeError("RTSP 打开失败")
                with self._cv:
                    self._connected = True
                while not self._stop.is_set():
                    ok, img = cap.read()
                    if self._stop.is_set():
                        break
                    if not ok or img is None:
                        raise RuntimeError("RTSP 读取失败")
                    ts = time.monotonic()
                    with self._cv:
                        if self._stop.is_set():
                            break
                        self._discard_latest_locked()
                        self._fid += 1  # 实际解码序号，不按主循环次数编号
                        self._latest = Frame(self._fid, img, ts)
                        self._last_frame_ts = ts
                        self._last_error = None
                        self._cv.notify_all()
            except Exception as exc:
                with self._cv:
                    self._last_error = f"{type(exc).__name__}: {exc}"
                if not self._stop.is_set():
                    print(f"[视频] {type(exc).__name__}: {exc}，重连中…（控制已归零）")
            finally:
                with self._cv:
                    self._connected = False
                    self._discard_latest_locked()
                    self._cv.notify_all()
                if cap is not None:
                    try:
                        cap.release()
                    except Exception as exc:
                        with self._cv:
                            self._last_error = f"release {type(exc).__name__}: {exc}"
                        print(f"[视频] RTSP release {type(exc).__name__}: {exc}")
            if self._stop.wait(self.reconnect_s):
                break

    def read(self):
        # 首帧短暂等待；正常等待新帧避免轮询暂时无帧导致 false LOST。
        with self._cv:
            wait_s = 6.0 if self._first_read else 0.2
            self._first_read = False
            deadline = time.monotonic() + wait_s
            while not self._stop.is_set():
                if self._worker is None or not self._worker.is_alive():
                    return None
                now = time.monotonic()
                if self._latest is not None:
                    if now - self._latest.ts > self._max_age_s:
                        self._discard_latest_locked()
                    elif self._connected and self._latest.frame_id > self._delivered_fid:
                        self._delivered_fid = self._latest.frame_id
                        return self._latest
                remaining = deadline - now
                if remaining <= 0:
                    return None
                self._cv.wait(remaining)
            return None

    def stats(self):
        with self._cv:
            # 这是本地成功解码后的帧龄，不能当成相机到手机的端到端延迟。
            age = (None if self._last_frame_ts is None else
                   max(0.0, (time.monotonic() - self._last_frame_ts) * 1000.0))
            return {"source_decoded_frames": self._fid,
                    "dropped_frames": self._dropped,
                    "latest_age_ms": age,
                    "source_frame_id": self._fid,
                    "connected": self._connected,
                    "worker_alive": self._worker is not None and self._worker.is_alive(),
                    "last_error": self._last_error}

    def close(self):
        self._stop.set()
        with self._cv:
            self._connected = False
            self._discard_latest_locked()
            self._cv.notify_all()
        if self._worker is not None:
            # 最多等待现有 5 秒 open/read 超时；即便后端不遵守超时也不阻塞停机。
            self._worker.join(timeout=6.0)
            if self._worker.is_alive():
                print("[视频] RTSP 后端尚未退出，停止等待；连接仍由解码线程释放")


class MockSource:
    def __init__(self, width=1280, height=720):
        self.w, self.h, self._fid = width, height, 0

    def open(self): print("[mock] 视频源占位")

    def read(self):
        if not HAS_CV2:
            raise RuntimeError("mock 视频源也需要 opencv")
        self._fid += 1
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        return Frame(self._fid, img, time.monotonic())

    def close(self): pass


def create(cfg):
    c = cfg["video"]
    if c["source"] == "rtsp":
        return RTSPSource(c["rtsp_url"], c["reconnect_interval_s"], c["width"], c["height"])
    return MockSource(c["width"] or 1280, c["height"] or 720)
