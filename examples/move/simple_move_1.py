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

class ArmCommanderMoveExample():
    """ This example demonstrates the following:
        - Create a general commander (arm_commander) that uses the move_group named "panda_arm"
        - Spin the general commander in a new thread and wait for the end of the internal setup
        - The function move_to_position to move a robot_link (the end-effector) to a particular x, y, z
        - The default value of the current component value if one or two of the x, y, z components is not given 
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
        # send a move command
        arm_commander.move_to_position(x = 0.6, y = 0.0, z = 0.4, wait=True)
        arm_commander.reset_state()
        # send a move command
        arm_commander.move_to_position(x = 0.4, y = 0.2, wait=True)
        the_state = arm_commander.get_commander_state()
        if the_state == GeneralCommanderStates.SUCCEEDED:
            logger.info('The move was successful')
        elif the_state in [GeneralCommanderStates.ABORTED, GeneralCommanderStates.ERROR]:
            logger.error(f'Error: {arm_commander.get_error_code()}')
        arm_commander.reset_state()
                    
    def stop(self, *args, **kwargs):
        logger.info(f'The stop signal received')
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()