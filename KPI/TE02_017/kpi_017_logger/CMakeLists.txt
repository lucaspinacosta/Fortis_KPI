
cmake_minimum_required(VERSION 3.0.2)
project(kpi_017_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  rosgraph_msgs
  moveit_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/kpi_017_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
