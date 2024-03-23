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
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory
from arm_commander.states import GeneralCommanderStates

class ArmCommanderMoveExample():
    """ This example demonstrates the following:
        - The use of wait=False to specify asynchronous move commands
        - The use of a while loop to poll the command status and exit when the status is not BUSY
        - The function wait_for_busy_end() that has implemented the while loop  
    """
    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        # send a move command
        arm_commander.move_to_position(x = 0.6, y = 0.0, z = 0.4, wait=True)
        arm_commander.reset_state()
        # send a move command in an asynchronous manner, z is defaulted to the current z value
        arm_commander.move_to_position(x = -0.6, y = 0.2, wait=False)        
        while True:
            the_state = arm_commander.get_commander_state()
            if the_state not in [GeneralCommanderStates.BUSY]:
                break
            time.sleep(0.1)
        arm_commander.reset_state()
        # the while loop can be replaced by the following function call
        # arm_commander.wait_for_busy_end()
            
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()