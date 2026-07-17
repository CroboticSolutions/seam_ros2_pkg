#!/usr/bin/env bash
# Smoke-check physical FANUC CRX + RealSense stack before GUI work.
# Usage: FANUC_ROBOT_IP=192.168.1.100 bash scripts/smoke_fanuc_real.sh

set -euo pipefail

ROBOT_IP="${FANUC_ROBOT_IP:-192.168.1.100}"
JS_TOPIC="${FANUC_JOINT_STATES_TOPIC:-/joint_states}"
IMG_TOPIC="${FANUC_CAMERA_IMAGE_TOPIC:-/camera/camera/color/image_raw}"
POINTS_TOPIC="${FANUC_CAMERA_POINTS_TOPIC:-/camera/camera/depth/color/points}"
HZ_SAMPLES="${SMOKE_HZ_SAMPLES:-5}"

pass=0
fail=0

check() {
  local name="$1"
  shift
  if "$@"; then
    echo "[OK]   $name"
    pass=$((pass + 1))
  else
    echo "[FAIL] $name"
    fail=$((fail + 1))
  fi
}

echo "=== FANUC real smoke (ip=$ROBOT_IP) ==="

check "ping $ROBOT_IP" ping -c 2 -W 1 "$ROBOT_IP"

if command -v ros2 >/dev/null 2>&1; then
  check "joint_states publishers" bash -c \
    "ros2 topic info '$JS_TOPIC' 2>/dev/null | grep -q 'Publisher count: [1-9]'"
  check "camera image publishers" bash -c \
    "ros2 topic info '$IMG_TOPIC' 2>/dev/null | grep -q 'Publisher count: [1-9]'"
  check "camera points publishers" bash -c \
    "ros2 topic info '$POINTS_TOPIC' 2>/dev/null | grep -q 'Publisher count: [1-9]'"
  check "move_group node" bash -c \
    "ros2 node list 2>/dev/null | grep -q move_group"
  check "arm_api2 current_pose" bash -c \
    "ros2 topic info /arm/state/current_pose 2>/dev/null | grep -q 'Publisher count: [1-9]'"

  echo "--- optional hz (samples=$HZ_SAMPLES) ---"
  ros2 topic hz "$JS_TOPIC" --window "$HZ_SAMPLES" || true
  ros2 topic hz "$IMG_TOPIC" --window "$HZ_SAMPLES" || true
else
  echo "[SKIP] ros2 not on PATH"
fi

echo "=== result: $pass passed, $fail failed ==="
exit $((fail > 0 ? 1 : 0))
