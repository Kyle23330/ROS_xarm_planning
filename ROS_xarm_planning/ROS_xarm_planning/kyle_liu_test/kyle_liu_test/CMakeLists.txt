cmake_minimum_required(VERSION 3.0.2)
project(kyle_liu_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(CATKIN_DEPENDS
  rospy
)

catkin_install_python(PROGRAMS
  scripts/test.py
  	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
