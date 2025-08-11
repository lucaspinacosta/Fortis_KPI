# TEO2-005 Validation Kit (ROS 1 — Noetic)

**Goal**: Validate KPI for TEO2-005 — *Successful conversion of raw sensor data into ROS-compatible messages with > 99% success rate*.

This kit provides a **ROS 1 (Noetic)** validator node that:
- Watches configured **raw** topics and their expected **converted** topics.
- Matches messages by timestamp (with a configurable tolerance).
- Computes per-sensor and overall **success rate**.
- Exports a CSV log of events and a JSON summary.

## Quick Start
```bash
mkdir -p ~/catkin_ws/src
cp -r teo2_005_validator_ros1 ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Configure topics
# edit: teo2_005_validator_ros1/config/sensors.yaml

# Launch
roslaunch teo2_005_validator_ros1 validator.launch sensors_config:=config/sensors.yaml tolerance_ms:=50 log_dir:=/tmp/teo2_005_logs threshold:=99.0
```
Outputs (inside `log_dir`):
- CSV: `conversion_events_YYYYmmdd_HHMMSS.csv`
- JSON: `summary_YYYYmmdd_HHMMSS.json`

**Pass Criteria**: overall success_rate ≥ `threshold` (default 99%).