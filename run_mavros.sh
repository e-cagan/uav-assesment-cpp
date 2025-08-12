#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-14557}"

# ROS env (nounset patlamasın)
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash || true
set -u

URL="udp://:${PORT}@"
echo "[MAVROS] starting mavros_node with fcu_url=${URL}"

exec ros2 run mavros mavros_node --ros-args \
  -r __ns:=/mavros \
  -p fcu_url:="${URL}" \
  -p fcu_protocol:="v2.0" \
  -p tgt_system:=1 \
  -p tgt_component:=1
