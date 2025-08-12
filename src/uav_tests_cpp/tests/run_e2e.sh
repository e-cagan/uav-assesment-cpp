#!/usr/bin/env bash
set -euo pipefail

GTEST_BIN="${1:-}"
if [[ -z "$GTEST_BIN" || ! -x "$GTEST_BIN" ]]; then
  echo "[ERR] GTest binary path invalid: $GTEST_BIN"
  exit 1
fi

# ROS env (nounset yüzünden patlamasın)
set +u
source /opt/ros/humble/setup.bash || true
set -u

# REPO_ROOT CMake'ten geliyorsa kullan; yoksa tests/../../..../ ile bul
if [[ -z "${REPO_ROOT:-}" ]]; then
  REPO_ROOT="$(cd "$(dirname "$0")"/../../../.. && pwd)"
fi

SITL="${REPO_ROOT}/run_sitl.sh"
MAVROS="${REPO_ROOT}/run_mavros.sh"

if [[ ! -x "$SITL" ]];  then echo "[ERR] missing or not executable: $SITL";  exit 2; fi
if [[ ! -x "$MAVROS" ]]; then echo "[ERR] missing or not executable: $MAVROS"; exit 2; fi

SITL_LOG="/tmp/sitl_e2e.log"
MAVROS_LOG="/tmp/mavros_e2e.log"
: > "$SITL_LOG"; : > "$MAVROS_LOG"

echo "[INFO] Starting PX4 SITL (headless)"
( SITL_LOG="$SITL_LOG" "$SITL" e2e >"$SITL_LOG" 2>&1 & )
sleep 2

echo "[INFO] Starting MAVROS (udp://:14557@)"
( "$MAVROS" 14557 >"$MAVROS_LOG" 2>&1 & )

echo "[INFO] Waiting for MAVROS topics/services..."
ok=0
for i in $(seq 1 60); do
  set +u; source /opt/ros/humble/setup.bash || true; set -u
  if ros2 topic list 2>/dev/null | grep -q "/mavros/state"; then ok=1; break; fi
  sleep 0.5
done
if [[ "$ok" -ne 1 ]]; then
  echo "[ERR] MAVROS not ready in time"
  echo "--- SITL tail ---";   tail -n 100 "$SITL_LOG"   || true
  echo "--- MAVROS tail ---"; tail -n 100 "$MAVROS_LOG" || true
  exit 3
fi

# GTest’i koştur
"$GTEST_BIN"

# Temizlik
pkill -f px4_sitl_default 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f mavros_node 2>/dev/null || true
