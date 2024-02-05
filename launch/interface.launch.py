"""
A launch file for running the motion planning python api tutorial
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="abb_irb6700_200_260", package_name="abb_moveit2_config"
        )
        .robot_description(file_path="config/abb_irb6700_200_260.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("arm_commander")
            + "/config/motion_planning.yaml"
        )
        .to_moveit_configs()
    )

    moveit_py_node = Node(
        name="test",
        package="arm_commander",
        executable="entry_point",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            moveit_py_node,
        ]
    )
