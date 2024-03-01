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
import arm_commander.moveit_tools as moveit_tools

class ArmCommanderMoveExample():
    def __init__(self):
        rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        
        # send a move command specified in Pose
        xyzrpy = [0.2, 0.0, 0.5, 3.14, 0.0, 0.6]
        pose = moveit_tools.create_pose(xyzrpy)
        arm_commander.move_to_pose(pose, wait=True)
        arm_commander.reset_state()

        # send a move command specified in PoseStamped
        xyzrpy = [0.5, 0.1, 0.4, 3.14, 0.0, -0.6]
        pose_stamped = moveit_tools.create_pose_stamped(xyzrpy, arm_commander.get_world_reference_frame())        
        arm_commander.move_to_pose(pose_stamped, wait=True)
        arm_commander.reset_state()
                
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()