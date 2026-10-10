#!/usr/bin/env python3
import argparse
import os
import json
import signal
import sys
import threading
import time
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from anti_drone_interfaces.msg import Track, TrackArray
from ultralytics import YOLO
from std_msgs.msg import String
from fusion_bridge import FusionBridge, publication_payload

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib

DEVICE = "/dev/video0"
MODEL = os.environ.get("SMART_GIMBAL_MODEL_PATH", "/home/jetson/visual_gimbal_lab/models/multirotor-v5-blend060.pt")
RTSP_PORT = "8554"
RTSP_PATH = "/yolo_hw"
WIDTH, HEIGHT, FPS = 640, 480, 15
CONF = float(os.environ.get("SMART_GIMBAL_PUBLISH_CONF", "0.35"))
FRAME_ID = "camera_optical_frame"

class FrameStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.source_ns = 0
        self.source_mono = 0.0
        self.stop = threading.Event()
        self.error = ""

    def run(self):
        cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            self.error = "camera_open_failed:" + DEVICE
            return
        try:
            while not self.stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.error = "camera_read_failed"
                    time.sleep(0.05)
                    continue
                frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
                with self.lock:
                    self.frame = np.ascontiguousarray(frame)
                    self.seq += 1
                    self.source_ns = time.time_ns()
                    self.source_mono = time.monotonic()
                    self.error = ""
        finally:
            cap.release()

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, self.seq, self.source_ns, self.source_mono
            return self.frame.copy(), self.seq, self.source_ns, self.source_mono

class RtspFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, store):
        super().__init__()
        self.store = store
        self.started = time.monotonic()
        self.duration = Gst.SECOND // FPS
        self.appsrc = None
        self.frames = 0
        self.next_send = 0.0
        self.set_shared(True)

    def do_create_element(self, url):
        pipeline = (
            "( appsrc name=source is-live=true block=false format=time "
            "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 "
            "! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast "
            "bitrate=1200 key-int-max=%d bframes=0 "
            "! rtph264pay name=pay0 pt=96 )"
            % (WIDTH, HEIGHT, FPS, FPS)
        )
        return Gst.parse_launch(pipeline)

    def do_configure(self, media):
        self.frames = 0
        self.next_send = time.monotonic()
        self.appsrc = media.get_element().get_by_name("source")
        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.connect("need-data", self.on_need_data)

    def on_need_data(self, src, length):
        frame, seq, source_ns, source_mono = self.store.latest()
        if frame is None or time.monotonic() - source_mono > 0.5:
            time.sleep(0.05)
            return
        buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
        buf.fill(0, frame.tobytes())
        now = time.monotonic()
        time.sleep(max(0.0, self.next_send - now))
        self.next_send = max(self.next_send + 1.0 / FPS, time.monotonic())
        pts = self.frames * self.duration
        self.frames += 1
        buf.pts = pts
        buf.dts = pts
        buf.duration = self.duration
        src.emit("push-buffer", buf)

class TrackNode(Node):
    def __init__(self, store):
        super().__init__("ros_track_pt_node")
        self.store = store
        self.primary_pub = self.create_publisher(
            Track, "/counter_uav/tracking/primary_target", 10)
        self.tracks_pub = self.create_publisher(
            TrackArray, "/counter_uav/tracking/tracks", 10)
        self.det_pub = self.create_publisher(
            Detection2DArray, "/counter_uav/perception/detections", 10)
        self.fusion_pub = self.create_publisher(String, "/counter_uav/tracking/fusion_status", 10)
        self.model = YOLO(MODEL)
        self.device = 0
        self.seq = 0
        self.hits = 0
        self.lost = 0
        self.last_infer = 0.0
        self.last_seq = 0
        self.prev_box = None
        self.track_id = 0
        self.publish_lock = threading.RLock()
        self.last_publish_mono = 0.0
        self.bridge = FusionBridge(self.model, MODEL)
        self.config_report = self.bridge.config_report
        self.last_status_log = 0.0
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            "loaded model %s; device=%s; physical_motion=false" % (MODEL, self.device))
        self.get_logger().info("fusion_config=" + json.dumps(self.config_report, sort_keys=True))

    def ros_stamp(self, ns):
        if ns <= 0:
            return self.get_clock().now().to_msg()
        return Time(sec=int(ns // 1000000000), nanosec=int(ns % 1000000000))

    def watchdog(self):
        if time.monotonic()-self.last_publish_mono > 0.5:
            with self.publish_lock:
                self._publish_fusion(0, 0, {"accepted": False, "source": "NONE", "bbox": None,
                                          "events": ["WATCHDOG_NO_FRESH_RESULT"]})

    def _publish_fusion(self, frame_seq, source_ns, row):
        """Same Track topic; supplemental source metadata never changes class IDs.

        Kalman-only predictions are never primary/observed detections. FEAR is a
        measured tracking observation, with recent YOLO semantic verification.
        """
        payload = publication_payload(row)
        self.last_publish_mono = time.monotonic()
        self.seq += 1
        now = self.get_clock().now().to_msg()
        stamp = self.ros_stamp(source_ns) if source_ns else now
        m = Track()
        m.header.stamp, m.header.frame_id = stamp, FRAME_ID
        m.sequence = self.seq
        m.source_stamp = stamp
        m.valid_until = self.ros_stamp(source_ns+500000000) if payload['primary'] and source_ns else now
        if payload['primary']:
            if 'ACQUISITION_CONFIRMED' in row.get('events', []) and self.hits > 0:
                self.track_id += 1
            if self.track_id == 0:
                self.track_id = 1
            self.hits += 1
            self.lost = 0
            x, y, w, h = payload['bbox']
            m.track_id = self.track_id
            m.class_name = "MULTIROTOR"
            m.confidence = float(payload['score'])
            m.xmin_px, m.ymin_px, m.xmax_px, m.ymax_px = x, y, x+w, y+h
            m.center_u_px, m.center_v_px = x+w/2, y+h/2
            m.width_px, m.height_px = w, h
            m.state, m.is_primary = 1, True
        else:
            self.lost += 1
            m.state, m.is_primary = 2, False
        m.age_frames, m.lost_frames = self.hits, self.lost
        arr = TrackArray()
        arr.header, arr.sequence, arr.source_stamp, arr.valid_until = m.header, m.sequence, m.source_stamp, m.valid_until
        arr.image_width_px, arr.image_height_px = WIDTH, HEIGHT
        arr.tracks.append(m)
        arr.active_track_count = int(m.is_primary)
        detections = Detection2DArray()
        detections.header = m.header
        # Detection2D is reserved for actual YOLO observations, not FEAR or KF.
        if payload['primary'] and row.get('source') == 'YOLO':
            raw = row.get('raw_bbox') or payload['bbox']
            d = Detection2D()
            d.header = m.header
            d.bbox.center.position.x, d.bbox.center.position.y = raw[0]+raw[2]/2, raw[1]+raw[3]/2
            d.bbox.size_x, d.bbox.size_y = raw[2], raw[3]
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = "MULTIROTOR", float(payload['score'])
            d.results.append(hypothesis)
            detections.detections.append(d)
        status = dict(row)
        status.update(sequence=self.seq, frame_sequence=frame_seq, source_ns=source_ns,
                      primary=bool(m.is_primary), prediction_hidden=payload['prediction_hidden'],
                      display_bbox=payload['bbox'], config=self.config_report,
                      metrics=self.bridge.metrics, physical_motion=False)
        text = String()
        text.data = json.dumps(status, allow_nan=False, sort_keys=True)
        # Consumers bind supplemental provenance to the exact Track sequence.
        self.fusion_pub.publish(text)
        self.primary_pub.publish(m)
        self.tracks_pub.publish(arr)
        self.det_pub.publish(detections)
        if time.monotonic()-self.last_status_log >= 2:
            self.last_status_log = time.monotonic()
            self.get_logger().info("fusion_status=" + text.data)

    def publish(self, frame_seq, source_ns, detections):
        with self.publish_lock:
            self._publish(frame_seq, source_ns, detections)

    def _publish(self, frame_seq, source_ns, detections):
        self.last_publish_mono = time.monotonic()
        source_stamp = self.ros_stamp(source_ns)
        now = self.get_clock().now().to_msg()
        self.seq += 1
        arr = TrackArray()
        arr.header.stamp = source_stamp
        arr.header.frame_id = FRAME_ID
        arr.sequence = self.seq
        arr.image_width_px = WIDTH
        arr.image_height_px = HEIGHT
        arr.source_stamp = source_stamp
        arr.valid_until = self.ros_stamp(source_ns + 500000000) if detections else now
        out = Detection2DArray()
        out.header = arr.header

        for b, conf, label in detections:
            x1, y1, x2, y2 = [float(v) for v in b]
            d = Detection2D()
            d.header = arr.header
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = max(0.0, x2 - x1)
            d.bbox.size_y = max(0.0, y2 - y1)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(label)
            h.hypothesis.score = float(conf)
            d.results.append(h)
            out.detections.append(d)

        if detections:
            b, conf, label = max(detections, key=lambda z: z[1])
            x1, y1, x2, y2 = [float(v) for v in b]
            if self.prev_box is None or self.iou(self.prev_box, b) < 0.2:
                self.track_id += 1
                self.hits = 1
            else:
                self.hits += 1
            self.prev_box = np.asarray(b)
            self.lost = 0
            state = 1 if self.hits >= 3 else 0
            is_primary = state == 1
            track_id = self.track_id
        else:
            self.hits = 0
            self.prev_box = None
            self.lost += 1
            state = 2
            is_primary = False
            track_id = 0
            x1 = y1 = x2 = y2 = 0.0
            conf = 0.0
            label = ""

        m = Track()
        m.header = arr.header
        m.sequence = self.seq
        m.track_id = track_id
        m.class_name = str(label)
        m.confidence = float(conf)
        m.center_u_px = (x1 + x2) / 2.0
        m.center_v_px = (y1 + y2) / 2.0
        m.width_px = max(0.0, x2 - x1)
        m.height_px = max(0.0, y2 - y1)
        m.xmin_px, m.ymin_px = x1, y1
        m.xmax_px, m.ymax_px = x2, y2
        m.age_frames = self.hits
        m.lost_frames = self.lost
        m.state = state
        m.is_primary = is_primary
        m.source_stamp = source_stamp
        m.valid_until = arr.valid_until
        arr.tracks.append(m)
        arr.active_track_count = 1 if is_primary else 0
        self.primary_pub.publish(m)
        self.tracks_pub.publish(arr)
        self.det_pub.publish(out)

        if self.seq == 1 or self.seq % 10 == 0:
            self.get_logger().info(
                "published primary_target seq=%d state=%d primary=%s conf=%.3f boxes=%d" %
                (self.seq, state, str(is_primary), float(conf), len(detections)))

    @staticmethod
    def iou(a, b):
        x1, y1 = max(a[0], b[0]), max(a[1], b[1])
        x2, y2 = min(a[2], b[2]), min(a[3], b[3])
        inter = max(0.0, x2-x1) * max(0.0, y2-y1)
        union = max(0.0, a[2]-a[0]) * max(0.0, a[3]-a[1]) + max(0.0, b[2]-b[0]) * max(0.0, b[3]-b[1]) - inter
        return inter / union if union > 0 else 0.0

    def step(self):
        frame, frame_seq, source_ns, source_mono = self.store.latest()
        now = time.monotonic()
        if now - self.last_infer < 1.0/FPS:
            return
        if frame is None or time.monotonic() - source_mono > 0.5:
            self.last_infer = now
            with self.publish_lock:
                self._publish_fusion(frame_seq, 0, {"accepted": False, "source": "NONE", "bbox": None,
                                                  "events": ["CAMERA_FRAME_MISSING_OR_STALE"]})
            return
        if frame_seq == self.last_seq:
            return
        self.last_seq = frame_seq
        self.last_infer = now
        row = self.bridge.process(frame, frame_seq, source_mono, now=now)
        if time.monotonic() - source_mono >= 0.5:
            row = dict(row, accepted=False, bbox=None, events=row.get('events',[])+['RESULT_EXPIRED_AT_COMPLETION'])
        with self.publish_lock:
            self._publish_fusion(frame_seq, source_ns, row)

def main():
    Gst.init(None)
    store = FrameStore()
    cap_thread = threading.Thread(target=store.run, name="v4l2-capture", daemon=True)
    cap_thread.start()
    time.sleep(0.5)

    server = GstRtspServer.RTSPServer()
    server.set_service(RTSP_PORT)
    factory = RtspFactory(store)
    server.get_mount_points().add_factory(RTSP_PATH, factory)
    server.attach(None)
    loop = GLib.MainLoop()
    gst_thread = threading.Thread(target=loop.run, name="rtsp-mainloop", daemon=True)
    gst_thread.start()

    rclpy.init()
    node = TrackNode(store)
    stop = threading.Event()
    def spin():
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    spin_thread = threading.Thread(target=spin, name="ros-watchdog", daemon=True)
    spin_thread.start()

    def shutdown(signum, frame):
        stop.set()
        store.stop.set()
        try:
            loop.quit()
        except Exception:
            pass

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    node.get_logger().info(
        "rtsp=rtsp://127.0.0.1:%s%s camera=%s physical_motion=false" %
        (RTSP_PORT, RTSP_PATH, DEVICE))

    try:
        while not stop.is_set():
            node.step()
            time.sleep(0.02)
    finally:
        stop.set()
        store.stop.set()
        loop.quit()
        spin_thread.join(timeout=1.0)
        with node.publish_lock:
            node._publish_fusion(0, 0, {"accepted": False, "source": "NONE", "bbox": None,
                                       "events": ["NODE_SHUTDOWN"]})
        node.bridge.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cap_thread.join(timeout=2.0)

def check(args):
    # Independent subscriber + RTSP decoder. No V4L2, model or actuator access.
    rclpy.init()
    node = Node("runtime_acceptance_sub")
    records = []
    def callback(msg):
        records.append({
            "sequence": int(msg.sequence), "track_id": int(msg.track_id),
            "state": int(msg.state), "is_primary": bool(msg.is_primary),
            "class_name": msg.class_name, "confidence": float(msg.confidence),
            "lost_frames": int(msg.lost_frames),
            "source_ns": int(msg.source_stamp.sec)*1000000000+msg.source_stamp.nanosec,
            "valid_until_ns": int(msg.valid_until.sec)*1000000000+msg.valid_until.nanosec,
            "received_ns": time.time_ns()})
    node.create_subscription(Track, args.topic, callback, 10)
    cap = cv2.VideoCapture(args.source, cv2.CAP_FFMPEG,
                          [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 12000,
                           cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000])
    frames, shape = 0, None
    end = time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if cap.isOpened():
                ok, image = cap.read()
                if ok:
                    frames += 1
                    shape = list(image.shape)
        good = frames>0 and len(records)>1 and records[-1]["sequence"]>records[0]["sequence"]
        result = {
            "status": "VIDEO_AND_PRIMARY_RECEIVED" if good else "NOT_ACCEPTED",
            "scope": "VIDEO_AND_REAL_MODEL_ROS_PUBLICATION_NOT_ACCURACY_OR_MOTION",
            "rtsp_frames": frames, "shape": shape,
            "primary_messages": len(records),
            "valid_primary_messages": sum(x["is_primary"] and x["valid_until_ns"]>x["received_ns"] for x in records),
            "first": records[0] if records else None,
            "last": records[-1] if records else None,
            "physical_motion": False}
        print(json.dumps(result,sort_keys=True),flush=True)
        return 0 if good else 2
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("action", nargs="?", choices=["run","check"], default="run")
    parser.add_argument("--seconds",type=float,default=8)
    parser.add_argument("--source",default="rtsp://127.0.0.1:8554/yolo_hw")
    parser.add_argument("--topic",default="/counter_uav/tracking/primary_target")
    args=parser.parse_args()
    try:
        if args.action=="check":
            sys.exit(check(args))
        main()
    except Exception:
        traceback.print_exc()
        raise
