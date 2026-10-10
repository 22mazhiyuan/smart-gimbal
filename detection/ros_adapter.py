"""ROS 2 检测适配：订阅 /counter_uav/tracking/primary_target -> DetectionResult。

rclpy 在后台线程 spin，主循环照常跑，不阻塞。
visible 判定：is_primary 且 lost_frames==0 且置信度达标 且 valid_until 未过期。
du/dv = 目标中心 - 画面中心（画面尺寸按视频帧实际宽高，不写死）。
"""
import os
import threading
import time

from detection.interface import BaseDetector, DetectionResult


class RosTrackDetector(BaseDetector):
    def __init__(self, img_w=0, img_h=0,
                 topic="/counter_uav/tracking/primary_target",
                 conf_thresh=0.4, ros_domain_id=99):
        self.img_w, self.img_h = img_w, img_h
        self.topic = topic
        self.conf_thresh = conf_thresh
        self.ros_domain_id = ros_domain_id
        self._latest = None
        self._lock = threading.Lock()
        self._node = None
        self._spin_stop = threading.Event()
        self._spin_thread = None
        self._rclpy = None
        self._owns_context = False

    def open(self):
        os.environ.setdefault("ROS_DOMAIN_ID", str(self.ros_domain_id))
        try:
            import rclpy
            from rclpy.node import Node
        except ImportError:
            raise RuntimeError("没装 rclpy")
        try:
            from anti_drone_interfaces.msg import Track
        except ImportError:
            raise RuntimeError("没找到 anti_drone_interfaces，先 colcon build 装好")
        try:
            from rclpy.signals import SignalHandlerOptions
            if not rclpy.ok():
                # main.py owns SIGINT/SIGTERM so timeout/systemd can stop it.
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
                self._owns_context = True
        except RuntimeError:
            pass  # 已经初始化过（比如被别的节点 init 了）
        node = Node("smart_gimbal_track_sub")

        def cb(msg):
            with self._lock:
                self._latest = msg

        node.create_subscription(Track, self.topic, cb, 10)
        self._node = node
        self._rclpy = rclpy
        self._spin_stop.clear()

        def spin():
            from rclpy.executors import ExternalShutdownException, ShutdownException
            try:
                while not self._spin_stop.is_set() and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
            except (ExternalShutdownException, ShutdownException):
                pass

        self._spin_thread = threading.Thread(target=spin, name="smart-gimbal-ros-spin", daemon=True)
        self._spin_thread.start()
        print(f"[ROS] 已订阅 {self.topic}（ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}，后台线程 spin）")

    def close(self):
        self._spin_stop.set()
        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._node:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy and self._rclpy.ok():
            self._rclpy.shutdown()
        self._owns_context = False

    def _fresh(self, msg) -> bool:
        """valid_until 过期检查：过期即视为不可见（TTL 问题修复）。"""
        vu = getattr(msg, "valid_until", None)
        if vu is None:
            return True
        try:
            t = float(vu.sec) + float(vu.nanosec) * 1e-9
            if t <= 0:
                return True  # 发布方没填就当有效
            return t >= time.time() - 1.0  # 1 秒宽限
        except Exception:
            return True

    def process(self, frame) -> DetectionResult:
        with self._lock:
            msg = self._latest
        if msg is None:
            return DetectionResult(frame_id=frame.frame_id, visible=False)
        try:
            conf = float(getattr(msg, "confidence", 0.0) or 0.0)
            valid = bool(getattr(msg, "is_primary", False)) \
                and int(getattr(msg, "lost_frames", 1) or 0) == 0 \
                and conf >= self.conf_thresh \
                and self._fresh(msg)
        except Exception:
            valid, conf = False, 0.0
        if not valid:
            return DetectionResult(frame_id=frame.frame_id, visible=False, conf=conf)
        # A reconnect may change image dimensions; use the current real frame.
        h, w = frame.image.shape[:2]
        self.img_w, self.img_h = w, h
        cu = float(msg.center_u_px)
        cv = float(msg.center_v_px)
        seq = int(getattr(msg, "sequence", 0) or frame.frame_id)
        return DetectionResult(
            frame_id=seq, visible=True,
            bbox_xyxy=(float(msg.xmin_px), float(msg.ymin_px),
                       float(msg.xmax_px), float(msg.ymax_px)),
            label=str(getattr(msg, "class_name", "") or ""),
            conf=conf, du_px=cu - w / 2.0, dv_px=cv - h / 2.0)


def create_detector(cfg, img_w=0, img_h=0):
    m = cfg["model"]
    # 环境变量覆盖，避免改被 git 跟踪的 config.yaml 产生冲突
    backend = os.environ.get("SMART_GIMBAL_DETECTOR", m["backend"])
    if backend == "ros_track":
        return RosTrackDetector(img_w, img_h,
                                m.get("ros_topic", "/counter_uav/tracking/primary_target"),
                                m.get("conf_thresh", 0.4),
                                m.get("ros_domain_id", 99))
    from detection.interface import MockDetector
    return MockDetector(width=img_w or 1280, height=img_h or 720)
