#!/usr/bin/env python3
# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

__author__ = 'Andrew Lui'
__author__ = 'Dasun Gunasinghe'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.0.1'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

# general modules
import signal, sys, rclpy, ament_index_python, threading

# project modules
from ref_moveit_interface.commander import ArmCommander
from moveit.planning import MoveItPy
from rclpy.logging import get_logger

__path__ = ament_index_python.packages.get_package_share_directory('ref_moveit_interface')


class CommanderDemo():
    """A demo program for the :class::'Commander', which illustrates commands offered by
    the Commander class
    """
    def __init__(self) -> None:
        rclpy.init(args=None)
        self.nh = rclpy.create_node(node_name="moveit_interface_node")
        self.logger = get_logger("moveit_interface_node")

        self.commander: ArmCommander = ArmCommander(moveit_group_name='manipulator', world_link='base_link', nh=self.nh)
        self.execution_thread = threading.Thread(target=self.run_demo, daemon=True)
        self.execution_thread.start()
        self.commander.spin()

    def run_demo(self):
        try:
            # Validate setup
            self.logger.info(f"Move Group: {self.commander.is_move_group_valid()}")
            self.logger.info(f"EE Link: {self.commander.is_ee_link_valid()}")
            self.logger.info(self.commander.info())

            # Via ROS transform buffer
            xyzq = self.commander.pose_in_frame_as_xyzq(link_name='tool0')
            xyzrpy = self.commander.pose_in_frame_as_xyzrpy(link_name='tool0')
            pose_stamped = self.commander.pose_in_frame(link_name='tool0')
            self.logger.info(f"XYZQ: {xyzq}")
            self.logger.info(f"XYZRPY: {xyzrpy}")
            self.logger.info(f"POSE STAMPED: {pose_stamped}")

            # Specifically from robot
            pose_of_robot_link = self.commander.pose_of_robot_link(robot_link_name='tool0')
            rpy_of_robot_link = self.commander.rpy_of_robot_link(robot_link_name='tool0')
            xyzrpy_of_robot_link = self.commander.xyzrpy_of_robot_link(robot_link_name='tool0')
            self.logger.info(f"POSE OF ROBOT LINK: {pose_of_robot_link}")
            self.logger.info(f"RPY OF ROBOT LINK: {rpy_of_robot_link}")
            self.logger.info(f"XYZRPY OF ROBOT LINK: {xyzrpy_of_robot_link}")

            # Attempt to move to Home pose
            self.commander.move_to_named_pose(named_pose="home", wait=False)

            self.commander.shutdown()
        except Exception as e:
            self.logger.error(e)

# -- callback function for shutdown
def cb_shutdown():
    sys.exit(0)

def stop(*args, **kwargs):
    sys.exit(0)
    
def main(args = None):
    """Main node entrypoint

    :param args: ROS2 launch arguments, defaults to None
    :type args: Any, optional
    """
    demo = CommanderDemo()

# -- the main program
if __name__ == '__main__':
    main()