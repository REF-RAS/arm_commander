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
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory, logger
from arm_commander.states import GeneralCommanderStates

class ResetRobotExample():
    """ This example demonstrates moving the robot arm back to a proper state
    """

    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        arm_commander.reset_world()
        # send a move command to move to a joint pose
        arm_commander.move_to_joint_pose([0.00, -1.243, 0.00, -2.949, 0.00, 1.704, 0.785], wait=True)
        logger.loginfo(f'The robot arm has returned to the stow pose')
        arm_commander.reset_state()
                    
    def stop(self, *args, **kwargs):
        logger.info(f'The stop signal received')
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ResetRobotExample()