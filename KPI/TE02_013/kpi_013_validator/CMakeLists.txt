
cmake_minimum_required(VERSION 3.0.2)
project(kpi_013_validator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/kpi_013_pose_accuracy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
