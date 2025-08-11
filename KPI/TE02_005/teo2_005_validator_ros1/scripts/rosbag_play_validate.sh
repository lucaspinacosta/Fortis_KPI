#!/usr/bin/env bash
set -e
if [ $# -lt 1 ]; then
  echo "Usage: $0 <bagfile> [log_dir]"
  exit 1
fi

BAG="$1"
LOG_DIR="${2:-/tmp/teo2_005_logs}"

echo "[bag] Playing $BAG with --clock"
rosbag play "$BAG" --clock &

sleep 2
echo "[validator] Launching validator"
roslaunch teo2_005_validator_ros1 validator.launch log_dir:="$LOG_DIR"