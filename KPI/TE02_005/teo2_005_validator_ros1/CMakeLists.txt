cmake_minimum_required(VERSION 3.0.2)
project(teo2_005_validator_ros1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/conversion_validator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)