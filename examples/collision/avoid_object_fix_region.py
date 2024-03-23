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
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory
import arm_commander.moveit_tools as moveit_tools

class ArmCommanderCollisionAvoidExample():
    """ This example demonstrates the following:
        - The creation of two positional path constraint objects to restrict the allowable regions that the end-effector can move within
        - The addition of two different positional path constraints to the general commander and their effect on the paths.
    """
    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.reset_world()  # to remove any workspace or object previously defined
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        arm_commander.clear_path_constraints() 
        
        # move to a starting position
        xyzrpy = [0.4, -0.15, 0.4, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(moveit_tools.create_pose(xyzrpy), wait=True)
        arm_commander.reset_state() 
        
        # add a collision object
        arm_commander.add_sphere_to_scene('the_sphere', 0.05, [0.4, 0.0, 0.4])   

        # add an positional path constraint to restrict the end-effector to move within the bbox below, which is the nearer side of the object
        allowed_bbox = [0.1, -0.3, 0.3, 0.5, +0.3, 0.6]
        arm_commander.add_path_constraints(moveit_tools.create_position_constraint_from_bbox(arm_commander.get_end_effector_link(), 
                    arm_commander.get_world_reference_frame(), allowed_bbox))
        
        # move to the opposite side of the object
        xyzrpy = [0.4, 0.15, 0.4, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(moveit_tools.create_pose(xyzrpy), wait=True)
        arm_commander.reset_state() 
        
        # clear the current positional path constraint and add another positional path constraint
        # to restrict the end-effector to move within the bbox below, which is the outer side of the object
        arm_commander.clear_path_constraints()   
        allowed_bbox = [0.3, -0.3, 0.3, 0.8, +0.3, 0.6]
        arm_commander.add_path_constraints(moveit_tools.create_position_constraint_from_bbox(arm_commander.get_end_effector_link(), 
                    arm_commander.get_world_reference_frame(), allowed_bbox))           

        # move to above the object
        xyzrpy = [0.4, -0.15, 0.4, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(moveit_tools.create_pose(xyzrpy), wait=True)
        arm_commander.reset_state()    
        
        arm_commander.clear_path_constraints()     
                    
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        self.arm_commander.clear_path_constraints() 
        self.arm_commander.reset_world()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderCollisionAvoidExample()