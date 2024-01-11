#!/usr/bin/env python3
# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

__author__ = 'Andrew Lui'
__author__ = 'Dasun Gunasinghe'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.1.0'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

# --- General Imports
import sys, copy, threading, time, signal, math, rospy

# --- Import Moveit Commander (Official)
import moveit_commander
import moveit_commander.conversions as conversions

# --- Import ROS Messages
from moveit_msgs.msg import MoveGroupActionFeedback, PlanningScene, ObjectColor 
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from std_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped

# Import State Definitions
from ref_moveit_interface.states import MoveitActionStates, RobotAgentStates

class MoveitInterface():
    def __init__(self) -> None:
        # create lock for synchronization
        self.action_lock = threading.Lock()

        # --- initialize Moveit! variables 
        self.robot = moveit_commander.RobotCommander()
        self.scene: moveit_commander.PlanningSceneInterface = moveit_commander.PlanningSceneInterface()

        # --- These attributes need configuring prior to use (may be different based on project)
        self.move_group: moveit_commander.MoveGroupCommander = None         
        self.end_effector_link = None 

        # --- ROS Subscribers
        # TODO: check if this name can change
        self.moveit_feedback_sub = rospy.Subscriber(
            name='/move_group/feedback', 
            data_class=MoveGroupActionFeedback, 
            callback=self.cb_move_group_feedback, 
            queue_size=1
        )
        self.agent_state: RobotAgentStates = RobotAgentStates.READY
        self.action_state: MoveitActionStates = MoveitActionStates.IDLE
        self.cached_feedback: MoveGroupActionFeedback = None

        # --- ROS Publishers
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

    # --- ROS Subscriber Callbacks
    def cb_move_group_feedback(self, msg: MoveGroupActionFeedback) -> None:
        """Subscriber callback to the move_group feedback

        :param msg: Move group action feedback message if published
        :type msg: MoveGroupActionFeedback
        """
        self.cached_feedback = msg
        if self.agent_state == RobotAgentStates.BUSY:
            # --- Handle status message
            if msg.status.status in [GoalStatus.SUCCEEDED]:
                rospy.loginfo(f'[MoveitInterface::cb_move_group_feedback][Goal Ended: {msg.status.text}]')
                self.agent_state = RobotAgentStates.SUCCEEDED
            elif msg.status.status in [GoalStatus.ABORTED]:
                rospy.loginfo(f'[MoveitInterface::cb_move_group_feedback][Goal Ended: {msg.status.text}]')
                self.agent_state = RobotAgentStates.ABORTED

            # --- Handle feedback state
            if msg.feedback.state == 'PLANNING':
                self.action_state = MoveitActionStates.PLANNING
            elif msg.feedback.state == 'MONITOR':
                self.action_state = MoveitActionStates.MONITOR
            elif msg.feedback.state == 'IDLE':
                if msg.status.text == 'TIMED_OUT':
                    self.action_state = MoveitActionStates.ERROR
                else:
                    self.action_state = MoveitActionStates.COMPLETED 

        return None
    
    # --- Get Methods
    def get_latest_feedback(self) -> MoveGroupActionFeedback:
        """Gets the latest feedback message cached from callback

        :return: Move group cached feedback message
        :rtype: MoveGroupActionFeedback
        """
        return self.cached_feedback
    
    def get_agent_state(self, print: bool = False) -> RobotAgentStates:
        """Gets the agent's state

        :param print: Prints the agent state name if True, defaults to False
        :type print: bool, optional
        :return: Robot agent state
        :rtype: RobotAgentStates
        """
        if print: rospy.loginfo(f"[MoveitInterface::set_ee_link][name: {self.agent_state.name}]")
        return self.action_state

    # --- Set Methods
    def set_ee_link(self) -> bool:
        """Sets the end-effector (ee) link through move_group

        :return: True if successful, else False
        :rtype: bool
        """
        # check if move_group has been set
        if self.is_move_group_valid():
            self.end_effector_link = self.move_group.get_end_effector_link()
            rospy.loginfo(f"[MoveitInterface::set_ee_link][Successfully set ee link: {self.end_effector_link}]")
        else:
            rospy.logerr(f"[MoveitInterface::set_ee_link][move_group not valid, cannot set ee link]")
            return False

    def set_move_group(self, group_name: str = "") -> bool:
        """Sets the move group attribute on provided group name

        :param group: name of move_group, defaults to ""
        :type group: str, optional
        :return: True if successful, otherwise False
        :rtype: bool
        """
        if group_name == "" or group_name == None:
            rospy.logerr(f"[MoveitInterface::set_move_group][Invalid group name given: {group_name}]")
            return False
        
        if self.move_group == None:
            # If not set, create a move group commander with group name provided
            self.move_group = moveit_commander.MoveGroupCommander(group_name)
            rospy.loginfo(f"[MoveitInterface::set_move_group][Successfully set move_group: {group_name}]")
            return True
        else:
            rospy.logerr(f"[MoveitInterface::set_move_group][move_group already set]")
            return False

    # --- Error Checking Methods
    def is_ee_link_valid(self) -> bool:
        """Confirms if an end-effector link has been set

        :return: True if set, or False
        :rtype: bool
        """
        if self.end_effector_link == None:
            return False
        else:
            return True
        
    def is_move_group_valid(self) -> bool:
        """Confirms if a move_group has been initialised
        - Returns True if set, or False

        :return: True or False depending on if move_group has been set
        :rtype: bool
        """
        if self.move_group == None:
            return False
        else:
            return True