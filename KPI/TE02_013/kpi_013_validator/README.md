
# KPI 013 Pose Accuracy Validator (ROS Noetic)

Validates KPI 013: "Estimates the robot’s position and orientation" by comparing
estimated pose to ground truth and checking that position error stays below the configured threshold.

## Features
- Works with `nav_msgs/Odometry` or `geometry_msgs/PoseWithCovarianceStamped`.
- Computes **position error** (m) and **orientation yaw error** (deg).
- Logs results to CSV and generates a JSON summary with pass/fail.
- Configurable via YAML and launch file.

## Quick Start
```bash
cd ~/catkin_ws/src
# Copy this folder here
catkin_make
source ~/catkin_ws/devel/setup.bash

roslaunch kpi_013_validator kpi_013.launch
```

## Config
Edit `config/kpi_013.yaml` to set:
- `estimated_topic`: ROS topic for estimated pose
- `ground_truth_topic`: ROS topic for ground truth pose
- `position_threshold_m`: KPI threshold for position error
- `log_dir`: where to save CSV/JSON logs

