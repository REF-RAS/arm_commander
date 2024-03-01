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

import sys, os, threading, signal, yaml
import rospy
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory

class ArmCommanderNamedPosesExample():
    """ This example demonstrates the following:
        - The setup of named joint-space poses
        - The use of a config file to specify named poses
        - The function move_to_named_pose() to move to a specified named pose
    """
    def __init__(self, config_file='panda_demo.yaml'):
        rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # create the General Commander and wait for it being ready to service move commands
        arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander.spin(spin_in_thread=True)
        arm_commander.wait_for_ready_to_move()
        self.arm_commander = arm_commander
        # load config file and install the named poses
        try:
            with open(os.path.join(os.path.dirname(__file__), f'{config_file}'), 'r') as f:
                self.demo_config = yaml.safe_load(f)
            for named_pose in self.demo_config['scene']['named_poses'].keys():
                arm_commander.add_named_pose(named_pose, self.demo_config['scene']['named_poses'][named_pose])
        except:
            rospy.logerr(f'Error in loading demo config file {config_file}')
            raise

        # wait for the General Commander ready to service move commands
        arm_commander.wait_for_ready_to_move()
        # send a move command to 'stow'
        rospy.loginfo(f'Move_to_named_pose: stow')
        arm_commander.move_to_named_pose('stow', wait=True)
        arm_commander.reset_state()
        # send a move command to 'home'
        rospy.loginfo(f'Move_to_named_pose: home')
        arm_commander.move_to_named_pose('home', wait=True)
        arm_commander.reset_state()
        
                    
    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderNamedPosesExample()