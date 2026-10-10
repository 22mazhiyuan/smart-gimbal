#!/bin/bash
# 开机自启安装：在 Jetson 上执行 sudo bash deploy/install_autostart.sh
set -euo pipefail
APP=/home/jetson/visual_gimbal_lab/smart_gimbal
DEPLOY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

install -m 0644 "$DEPLOY_DIR/smart_gimbal.service" /etc/systemd/system/smart_gimbal.service
install -d -m 0755 /etc/systemd/system/smart_gimbal.service.d
install -m 0644 "$DEPLOY_DIR/ros_track.override.conf" /etc/systemd/system/smart_gimbal.service.d/ros_track.conf
systemctl daemon-reload
systemctl enable smart_gimbal

echo "=== 自检（只看结果，不拦安装）==="
sudo -u jetson /bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/anti_drone_ws/install/setup.bash && export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 GIMBAL_REAL_MOTION=0 SMART_GIMBAL_DETECTOR=ros_track && cd "$1" && python3 bringup.py' bash "$APP" || true

echo "=== 启动服务 ==="
systemctl restart smart_gimbal
sleep 3
systemctl is-active smart_gimbal || echo "服务未起来，看日志：journalctl -u smart_gimbal -n 50"
echo "=== 生效单元（含 ros_track.conf override）==="
systemctl cat smart_gimbal

echo ""
echo "画面：http://<jetson-ip>:8080/video"
echo "日志：journalctl -u smart_gimbal -f"
echo "停用：sudo systemctl disable --now smart_gimbal"
