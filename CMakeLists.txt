# Created with QCR's code template tool: https://github.com/qcr/code_templates

cmake_minimum_required(VERSION 3.0.2)
project(arm_commander)

find_package(catkin REQUIRED COMPONENTS
  rospy
  )

catkin_python_setup()

catkin_package(
  )

catkin_install_python(PROGRAMS
  examples/commander_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

