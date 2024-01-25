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
import signal, sys, rclpy, ament_index_python

# project modules
from ref_moveit_interface.commander import ArmCommander
from moveit.planning import MoveItPy
from rclpy.logging import get_logger

__path__ = ament_index_python.packages.get_package_share_directory('ref_moveit_interface')

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
    rclpy.init(args=args)
    # Define the logging interface to use
    logger = get_logger("moveit_interface_node")
    node = rclpy.create_node(node_name="moveit_interface_node")
    signal.signal(signal.SIGINT, stop)

    try:
        logger.info('Test REF moveit interface node is running')
        # Currently needs a node input, really only for TF broadcasting/listening
        # TODO: investigate a way to peel this out possibly
        commander = ArmCommander(
            moveit_group_name="manipulator",
            world_link="base_link",
            nh=node
        )

        # Validate setup
        logger.info(f"Move Group: {commander.is_move_group_valid()}")
        logger.info(f"EE Link: {commander.is_ee_link_valid()}")

        logger.info(commander.info())

        # Get the current ee link (default) pose
        logger.info(f"EE link pose: {commander.get_current_link_pose()}")

        # Attempt to move to Home pose
        commander.move_to_named_pose(named_pose="home", wait=False)
        
    except Exception as e:
        logger.error(e)

    # Shutdown robot and node
    # commander.abort_move()
    commander.shutdown()
    rclpy.shutdown()

# -- the main program
if __name__ == '__main__':
    main()