#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI
# Robotics and Autonomous Systems Group
# Research Engineering Facility, Queensland University of Technology (QUT)
#
# License: https://github.com/REF-RAS/arm_commander/blob/main/LICENSE

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.0.1'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

import sys, signal
from arm_commander.commander_moveit import GeneralCommander, logger
from arm_commander.states import GeneralCommanderStates

class ArmCommanderMultiMoveExample():
    """ This example demonstrates the following:
        - Create a general commander (arm_commander) that uses the move_group named "panda_arm"
        - Spin the general commander in a new thread and wait for the end of the internal setup
        - The function move_to_multi_positions to move a robot_link (the end-effector) through a series of waypoints, defined as a list of xyz
        - The default value is the current component value if any of the x, y, z components is not given in the xyz list
        - The function reset_state() for clearing the state of the general commander and get ready for the next move command
    """

    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander:GeneralCommander = GeneralCommander('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        # create waypoints
        xyz_list = [(0.6, 0.0, 0.4), (0.6, 0.2, 0.5), (0.6, 0.2, 0.6), (0.6, 0.0, 0.7), 
                    (0.6, -0.2, 0.6), (0.6, -0.2, 0.5), (0.6, 0.0, 0.4)]
        # send a multi move command
        arm_commander.move_to_multi_positions(xyz_list=xyz_list, wait=True)
        the_state = arm_commander.get_commander_state()
        if the_state == GeneralCommanderStates.SUCCEEDED:
            logger.info('The first multi move was successful')
        elif the_state in [GeneralCommanderStates.ABORTED, GeneralCommanderStates.ERROR]:
            logger.info(f'Error: {arm_commander.get_error_code()}')
        arm_commander.reset_state()
        # send another multi move command
        xyz_list = [(None, 0.0, 0.4), (None, 0.2, 0.5), (None, 0.2, 0.6), (None, 0.0, 0.7), 
                    (None, -0.2, 0.6), (None, -0.2, 0.5), (None, 0.0, 0.4)]
        arm_commander.move_to_multi_positions(xyz_list=xyz_list, wait=True)
        the_state = arm_commander.get_commander_state()
        if the_state == GeneralCommanderStates.SUCCEEDED:
            logger.info('The second multi move was successful')
        elif the_state in [GeneralCommanderStates.ABORTED, GeneralCommanderStates.ERROR]:
            logger.info(f'Error: {arm_commander.get_error_code()}')
        
        arm_commander.reset_state()
                    
    def stop(self, *args, **kwargs):
        logger.info(f'The stop signal received')
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMultiMoveExample()