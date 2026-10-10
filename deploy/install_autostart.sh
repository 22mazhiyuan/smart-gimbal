#!/bin/bash
# 开机自启安装：在 Jetson 上执行 sudo bash deploy/install_autostart.sh
set -e
APP=/home/jetson/visual_gimbal_lab/smart_gimbal

cp "$(dirname "$0")/smart_gimbal.service" /etc/systemd/system/smart_gimbal.service
systemctl daemon-reload
systemctl enable smart_gimbal

echo "=== 自检（只看结果，不拦安装）==="
sudo -u jetson bash -c "cd $APP && python3 bringup.py" || true

echo "=== 启动服务 ==="
systemctl restart smart_gimbal
sleep 3
systemctl is-active smart_gimbal

echo ""
echo "画面：http://<jetson-ip>:8080/video"
echo "日志：journalctl -u smart_gimbal -f"
echo "停用：sudo systemctl disable --now smart_gimbal"
