#!/usr/bin/env bash
set -eo pipefail   # <— -u yok

# Kullanım: ./run_mavros.sh [PORT]
PORT="${1:-14557}"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  # ROS setup dosyası -u ile source edilemez, biz zaten -u kullanmıyoruz
  source /opt/ros/humble/setup.bash
fi

URL="udp://:${PORT}@"
echo "[MAVROS] starting mavros_node with fcu_url=${URL}"

exec ros2 run mavros mavros_node --ros-args \
  -r __ns:=/mavros \
  -p fcu_url:="${URL}" \
  -p fcu_protocol:="v2.0" \
  -p tgt_system:=1 \
  -p tgt_component:=1
