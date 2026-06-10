cmake_minimum_required(VERSION 3.0.2)
project(onboard_adapter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  quadrotor_msgs
  mavros_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/px4_setpoint_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
