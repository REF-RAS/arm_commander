# Created with QCR's code template tool: https://github.com/qcr/code_templates

cmake_minimum_required(VERSION 3.10.2)
project(ref_moveit_interface)

find_package(ament_cmake REQUIRED)

# ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

