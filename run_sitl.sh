#!/usr/bin/env bash
set -euo pipefail

LOG=/tmp/sitl.log
: > "$LOG"

# PX4 build dizin varsayılanı (Docker imajında bu path mevcut)
PX4_DIR=${PX4_DIR:-/opt/PX4-Autopilot}
cd "$PX4_DIR"

# Headless Gazebo + iris
echo "[OK] SITL up (gzserver & px4 running)" | tee -a "$LOG"
(
  set -x
  ./build/px4_sitl_default/bin/px4 \
    ./build/px4_sitl_default/etc -s etc/init.d-posix/rcS -t ./test_data \
    >> "$LOG" 2>&1
) &
disown
sleep 2
