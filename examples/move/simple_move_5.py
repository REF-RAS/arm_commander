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
import rospy
from arm_commander.commander_moveit import GeneralCommander, logger

class ArmCommanderMoveExample():
    """ This example demonstrates the following:
        - The parameter cartesian of the function move_to_position() specifies if cartesian path planning (with collision avoidance) is used
        - The difference between the paths coming from a path planner and cartesisn planner (moving in a straight line). 
    """
    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander:GeneralCommander = GeneralCommander('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        
        # send a move command to a xyz position and rpy orientation
        logger.info(f'Go to start')
        arm_commander.move_to_position(x = 0.0, y = -0.5, z = 0.2, wait=True)
        arm_commander.reset_state()
        arm_commander.rotate_to_orientation(roll = 3.14, pitch = 0.0, yaw = 0.2, wait=True)
        arm_commander.reset_state()
        logger.info(f'From start to target (cartesian is False)')
        arm_commander.move_to_position(x = 0.5, y = 0.0, z = 0.4, cartesian=False, wait=True)
        arm_commander.reset_state()
        # send a move command, z is defaulted to the current z value
        logger.info(f'Go back to start')
        arm_commander.move_to_position(x = 0.0, y = -0.5, z = 0.2, wait=True)            
        arm_commander.reset_state()   
        # send a move command moveing back to the original position, constrained cartesian movement
        logger.info(f'From start to target (cartesian is True)')
        arm_commander.move_to_position(x = 0.5, y = 0.0, z = 0.4, cartesian=True, wait=True)
        arm_commander.reset_state()        
                
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()