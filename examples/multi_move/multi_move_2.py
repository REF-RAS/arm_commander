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

import sys, threading, signal
import rospy
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory
from arm_commander.states import GeneralCommanderStates

class ArmCommanderMultiMoveExample():
    """ This example demonstrates the following:
        - Create a general commander (arm_commander) that uses the move_group named "panda_arm"
        - Spin the general commander in a new thread and wait for the end of the internal setup
        - The function move_to_multi_poses to move a robot_link (the end-effector) through a series of waypoints, defined as a list of 6-list,
          7-list, Pose or PoseStamped
        - The None value is not acceptable in any waypoint
        - The function reset_state() for clearing the state of the general commander and get ready for the next move command
    """

    def __init__(self):
        rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        # create waypoints
        xyzrpy_list = [(0.6, 0.0, 0.4, 3.14, 0, 0), 
                    (0.6, 0.2, 0.5, 3.14, 0, 0), 
                    (0.6, 0.2, 0.6, 3.14, 0, 1.58), 
                    (0.6, 0.0, 0.7, 3.14, 0, 3.14), 
                    (0.6, -0.2, 0.6, 3.14, 0, 1.58), 
                    (0.6, -0.2, 0.5, 3.14, 0, 0), 
                    (0.6, 0.0, 0.4, 3.14, 0, 0)]
        # send a multi move command
        arm_commander.move_to_multi_poses(waypoints_list=xyzrpy_list, wait=True)
        the_state = arm_commander.get_commander_state()
        if the_state == GeneralCommanderStates.SUCCEEDED:
            print('The multi move pose was successful')
        elif the_state in [GeneralCommanderStates.ABORTED, GeneralCommanderStates.ERROR]:
            print(f'Error: {arm_commander.get_error_code()}')
        arm_commander.reset_state()
                    
    def stop(self, *args, **kwargs):
        rospy.loginfo(f'stop received')
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMultiMoveExample()