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

class ArmCommanderWorkspaceExample():
    """ This example demonstrates the following:
        - The setup of the workspace, which composed of six walls, specified as a 3D bbox
        - The abort of move commander due to infringement of the workspace limits. 
    """
    def __init__(self):
        rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.reset_world()  # to remove any workspace or object previously defined
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        
        # move to a starting position
        arm_commander.move_to_position(x = 0.4, y = 0.0, z = 0.4, wait=True)
        arm_commander.reset_state()        
        # add the workspace as a bounding box of xmin, ymin, zmin, zmax, ymax, zmax
        workspace_bbox = [-0.4, -0.3, -0.2, 0.6, +0.3, 0.8]
        arm_commander.set_workspace_walls(*workspace_bbox)
        # move within the workspace
        arm_commander.move_to_position(x = 0.3, y = -0.1, z = 0.3, wait=True)
        arm_commander.reset_state()
        # move outside the workspace
        arm_commander.move_to_position(y = 0.35, wait=True)
        arm_commander.reset_state()
        
        arm_commander.reset_world()
                    
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        self.arm_commander.reset_world()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderWorkspaceExample()