# Created with QCR's code template tool: https://github.com/qcr/code_templates

cmake_minimum_required(VERSION 3.10.2)
project(arm_commander)

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)

# Install Python Package
# ament_python_install_package(arm_commander)

# Install Other Folders/Files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY arm_commander DESTINATION lib)

ament_package()
