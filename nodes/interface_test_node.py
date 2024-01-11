#!/usr/bin/env python3
# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

__author__ = 'Andrew Lui'
__author__ = 'Dasun Gunasinghe'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.0.1'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

# general modules
import signal, sys, rospy

# project modules
from ref_moveit_interface.interface import MoveitInterface

# -- callback function for shutdown
def cb_shutdown():
    sys.exit(0)

def stop(*args, **kwargs):
    sys.exit(0)
    
# -- the main program
if __name__ == '__main__':
    rospy.init_node('moveit_interface_test_node', anonymous=False)
    signal.signal(signal.SIGINT, stop)
    # rospy.on_shutdown(cb_shutdown)
    try:
        rospy.loginfo('Test REF moveit interface node is running')
        moveit_interface = MoveitInterface()
        # Setup the interface move_group
        moveit_interface.set_move_group(group_name="Manipulator")
        moveit_interface.set_ee_link()

        # Validate setup
        rospy.loginfo(f"Move Group: {moveit_interface.is_move_group_valid()}")
        rospy.loginfo(f"EE Link: {moveit_interface.is_ee_link_valid()}")

        # Get the current ee link (default) pose
        rospy.loginfo(f"EE link pose: {moveit_interface.get_current_link_pose()}")

        # Attempt to move to Home pose
        moveit_interface.move_to_named_pose(named_pose="home")
        # DO THINGS
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)