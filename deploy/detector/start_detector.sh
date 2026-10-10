#!/usr/bin/env bash
set -e
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source /opt/ros/humble/setup.bash
source /home/jetson/anti_drone_ws/install/setup.bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
export GIMBAL_REAL_MOTION=0
export MULTIROTOR_TRACKER_ROOT=/home/jetson/multirotor_tracker
export SMART_GIMBAL_CONFIG=/home/jetson/visual_gimbal_lab/smart_gimbal/config.yaml
export SMART_GIMBAL_MODEL_PATH="$HERE/model.pt"
export SMART_GIMBAL_PUBLISH_CONF=0.35
export PYTHONUNBUFFERED=1
cd "$HERE"
exec python3 "$HERE/video_detect_runtime.py" run
