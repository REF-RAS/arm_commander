# Created with QCR's code template tool: https://github.com/qcr/code_templates

cmake_minimum_required(VERSION 3.0.2)
project(ref_moveit_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  )

catkin_python_setup()

catkin_package(
  )

catkin_install_python(PROGRAMS
  nodes/interface_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

