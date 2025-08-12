
# KPI 017 Logger (ROS Noetic)

Validates KPI 017: "Utilises algorithms to plan and generate safe motion paths to avoid obstacles"
by monitoring the Manitou Velocity Controller logs and MoveIt trajectories.

## Features
- Watches `/rosout` for:
  - "Predicted collision"
  - "Collision detected"
  - "Replan succeeded"
- Records `/move_group/display_controller_planned_path` to capture before/after replans.
- Automatically records a rosbag of `/rosout` and `/move_group/display_controller_planned_path`.
- Pass criteria:
  1. At least one collision prediction/detection.
  2. At least one replan succeeded event after that.
  3. No collisions after the last replan.
- Stops automatically when pass/fail is determined.

## Usage
```bash
cd ~/catkin_ws/src
# Copy this folder here
catkin_make
source ~/catkin_ws/devel/setup.bash

roslaunch kpi_017_logger kpi_017.launch
```
Artifacts are saved to `/tmp/kpi_017_<timestamp>` by default.

