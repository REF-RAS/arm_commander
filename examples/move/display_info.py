#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI
# Robotics and Autonomous Systems Group
# Research Engineering Facility, Queensland University of Technology (QUT)
#
# License: https://github.com/REF-RAS/arm_commander/blob/main/LICENSE

__author__ = 'Andrew Lui'
__author__ = 'Dasun Gunasinghe'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.0.1'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

import sys, copy, threading, time, signal

from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory, logger

class ArmCommanderDisplayInfo():

    def __init__(self):
        # rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        logger.info(f'Creating the General Commander')
        self.arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
        arm_commander = self.arm_commander
        
        self.the_thread = threading.Thread(target=self.arm_commander.spin, daemon=True)
        self.the_thread.start()
        # initialize the GeneralCommander and print general information
        arm_commander.set_max_cartesian_speed(0.1)
        arm_commander.set_planning_time(10.0)
        arm_commander.reset_world()
                
        arm_commander.info(print=True)

    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        sys.exit(0)

if __name__=='__main__':
    ArmCommanderDisplayInfo()