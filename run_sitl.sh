#!/usr/bin/env bash
set -euo pipefail

# ROS env (nounset patlamasın)
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash || true
set -u

# PX4 kaynak dizini: Docker'da /opt/PX4-Autopilot, lokalde export edebilirsin
PX4_DIR="${PX4_DIR:-/opt/PX4-Autopilot}"
if [[ ! -d "$PX4_DIR" ]]; then
  echo "[ERR] PX4_DIR not found: $PX4_DIR"
  echo "Hint:  git clone https://github.com/PX4/PX4-Autopilot.git \$HOME/PX4-Autopilot"
  echo "       export PX4_DIR=\$HOME/PX4-Autopilot"
  exit 2
fi

LOG="${SITL_LOG:-/tmp/sitl.log}"
: > "$LOG"

cd "$PX4_DIR"

# Binary kontrol (lokalde derli değilse kullanıcıya ipucu ver)
if [[ ! -x ./build/px4_sitl_default/bin/px4 ]]; then
  echo "[ERR] PX4 SITL binary not found at $PX4_DIR/build/px4_sitl_default/bin/px4" | tee -a "$LOG"
  echo "Build it first: 'make px4_sitl gazebo' (inside $PX4_DIR)" | tee -a "$LOG"
  exit 3
fi

echo "[OK] SITL up (gzserver & px4 running)" | tee -a "$LOG"
(
  ./build/px4_sitl_default/bin/px4 \
    ./build/px4_sitl_default/etc -s etc/init.d-posix/rcS -t ./test_data \
    >> "$LOG" 2>&1
) & disown

sleep 2
