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

import sys, signal, time
from arm_commander.commander_moveit import GeneralCommander

class ArmCommanderMoveExample():
    """ This example demonstrates the following:
        - The function wait_for_busy_end() to turn an asynchronous move command into a blocking one
        - The function abort_move() for cancelling the current asynchronous move command 
        - The use of wait=True with the abort_move() command for a blocking cancal call (returns when the abort is complete)
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
        arm_commander.move_to_position(x = 0.6, y = 0.0, z = 0.4, wait=True)
        arm_commander.reset_state()
        # send a move command in an asynchronous manner, z is defaulted to the current z value
        arm_commander.move_to_position(x = -0.6, y = 0.2, wait=False)
        # abort the command after 3 seconds and wait for the abort to take effect
        time.sleep(3.0)
        arm_commander.abort_move(wait=True)
        arm_commander.reset_state()
        # send a move command, x and y are defaulted to the current values
        arm_commander.move_to_position(z = 0.3, wait=False) 
        arm_commander.wait_for_busy_end()
        arm_commander.reset_state()
                
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()