#!/usr/bin/env bash
set -euo pipefail

(cd /opt/PX4-Autopilot && HEADLESS=1 make px4_sitl gazebo >/tmp/sitl.log 2>&1 &)

for i in {1..30}; do
  if pgrep -x gzserver >/dev/null 2>&1 && pgrep -f px4 >/dev/null 2>&1; then
    echo "[OK] SITL up (gzserver & px4 running)"
    head -n 50 /tmp/sitl.log || true
    exit 0
  fi
  sleep 1
done

echo "[ERR] SITL failed to start. Log tail:"
tail -n 200 /tmp/sitl.log || true
exit 1
