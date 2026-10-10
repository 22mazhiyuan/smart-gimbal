#!/bin/bash
# Detection-service only; never installs or modifies actuator/IBVS/focus code.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo" >&2; exit 1; }
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
RUNTIME=/home/jetson/visual_gimbal_lab/multirotor_accuracy_repair
TRACKER=/home/jetson/multirotor_tracker
test -f "$RUNTIME/model.pt"
test -f "$TRACKER/fusion_core.py"
test -f "$TRACKER/models/FEAR-XS-NoEmbs.ckpt"
systemctl stop smart_gimbal.service
systemctl stop ros_track.service
for name in video_detect_runtime.py fusion_bridge.py start_detector.sh; do
  install -o jetson -g jetson -m 0644 "$ROOT/deploy/detector/$name" "$RUNTIME/$name"
done
chmod 0755 "$RUNTIME/start_detector.sh"
install -m 0644 "$ROOT/deploy/ros_track.service" /etc/systemd/system/ros_track.service
systemctl daemon-reload
systemctl enable ros_track.service
systemctl start ros_track.service
systemctl start smart_gimbal.service
echo "Services started. Check fusion_status and journal; active alone is not target acceptance."
systemctl is-active ros_track.service rtsp_stream.service smart_gimbal.service
