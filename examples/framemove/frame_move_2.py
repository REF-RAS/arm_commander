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
        - The creation of two collision objects as two reference frames
        - Movement to same relative position as the targets of move commands in two different reference frames
    """
    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)

        # create the General Commander and wait for it being ready to service move commands
        arm_commander:GeneralCommander = GeneralCommander('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.reset_world()
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        
        # move to a starting position
        xyzrpy = [0.4, 0.0, 0.4, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(xyzrpy, wait=True)
        arm_commander.reset_state() 
        
        # create two collision objects that act as the reference frame
        arm_commander.add_box_to_scene('area_1', [0.4, 0.2, 0.01], xyz=[0.5, 0.2, 0.2], rpy=[0, 0, 0])
        arm_commander.add_box_to_scene('area_2', [0.4, 0.2, 0.01], xyz=[0.5, -0.2, 0.1], rpy=[0, 0, 1.57])     
        
        # send a move command to a position in the frame of area_1
        arm_commander.move_to_position(x = 0.0, y = 0.0, z = 0.15, wait=True, reference_frame='area_1')
        arm_commander.reset_state()
        # send a move command to the same relative position but in the frame of area_2
        arm_commander.move_to_position(x = 0.0, y = 0.0, z = 0.15, wait=True, reference_frame='area_2')
        arm_commander.reset_state()        
        
        # send a move command to a position in the frame of area_1
        arm_commander.move_to_position(x = -0.2, y = -0.1, z = 0.15, wait=True, reference_frame='area_1')
        arm_commander.reset_state()      
        # send a move command to the same relative position but in the frame of area_2       
        arm_commander.move_to_position(x = -0.2, y = -0.1, z = 0.15, wait=True, reference_frame='area_2')
        arm_commander.reset_state()           

    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        self.arm_commander.reset_world()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderMoveExample()