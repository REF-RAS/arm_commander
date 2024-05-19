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
import arm_commander.tools.moveit_tools as moveit_tools

class ArmCommanderCollisionAvoidExample():
    """ This example demonstrates the following:
        - The addition of a collision object (sphere) to the world
        - The path of the end-effector moving between two locations resulting from avoiding collision with the object 
    """
    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander:GeneralCommander = GeneralCommander('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.reset_world()  # to remove any workspace or object previously defined
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        
        # move to a starting position
        xyzrpy = [0.4, -0.15, 0.4, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(xyzrpy, wait=True)
        arm_commander.reset_state() 
        
        # add a collision object
        arm_commander.add_sphere_to_scene('the_sphere', 0.05, [0.4, 0.0, 0.4])   

        # move to the opposite side of the object
        xyzrpy = [0.4, 0.15, 0.4, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(xyzrpy, wait=True)
        arm_commander.reset_state()         

        # move to the top of the object
        xyzrpy = [0.4, 0.0, 0.6, 3.14, 0.0, 0.6]
        arm_commander.move_to_pose(xyzrpy, wait=True)
        arm_commander.reset_state()    

        time.sleep(5)     
                    
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        self.arm_commander.reset_world()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderCollisionAvoidExample()