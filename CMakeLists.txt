# Created with QCR's code template tool: https://github.com/qcr/code_templates

cmake_minimum_required(VERSION 3.10.2)
project(arm_commander)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

# --- Handle Cpp Setup
# Include Cpp "include" directory
include_directories(arm_commander_cpp/include)

# Create Cpp executable
add_executable(cpp_executable arm_commander_cpp/commander_demo.cpp arm_commander_cpp/src/commander.cpp)
ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# --- Handle Python Setup
# Install Python Package
ament_python_install_package(arm_commander_py)

# Install Python executables
install(PROGRAMS
  arm_commander_py/commander_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Other Folders/Files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
