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
from arm_commander.commander_moveit import GeneralCommander

class ArmCommanderMoveExample():
    """ This example demonstrates the following:
        - The function rotate_to_orientation() to rotate the end-effector to different orientations specified in roll, pitch and yaw
        - The function supports parameter values defaulted to the current values
        - The function move_displacement() to move a displacement with respect to the current position.
    """
    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        
        # create the General Commander and wait for it being ready to service move commands
        arm_commander:GeneralCommander = GeneralCommander('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        
        # send a move command to a xyz position
        arm_commander.move_to_position(x = 0.3, y = -0.5, z = 0.2, wait=True)
        arm_commander.reset_state()
        arm_commander.rotate_to_orientation(roll = 3.14, pitch = 0.0, yaw = 0.2, wait=True)
        arm_commander.reset_state()
        
        for step in range(10):
            arm_commander.move_displacement(dy = 0.1, wait=True)
            arm_commander.reset_state()

        for step in range(5):
            arm_commander.move_displacement(dz = 0.1, wait=True)
            arm_commander.reset_state()
 
        for step in range(10):
            arm_commander.move_displacement(dy = -0.1, wait=True)
            arm_commander.reset_state()
            
        for step in range(5):
            arm_commander.move_displacement(dz = -0.1, wait=True)
            arm_commander.reset_state()            
            
                
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()