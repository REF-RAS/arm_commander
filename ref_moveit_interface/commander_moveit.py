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
import sys, copy, threading, time, signal, math, rclpy
import timeit

# --- Import moveit_py bindings to moveit core (Official)
# import moveit_commander
# import moveit_commander.conversions as conversions
from moveit.core.robot_state import RobotState
from moveit.core.controller_manager import ExecutionStatus
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


# --- Import ROS Messages
# from moveit_msgs.msg import MoveGroupActionFeedback, PlanningScene, ObjectColor 
# from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from std_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped

# Import State Definitions
from ref_moveit_interface.states import MoveitActionStates, RobotAgentStates

class CustomCommander():
    def __init__(self, robot=None, logger=None, moveit_group_name: str = None, world_link: str = None) -> None:
        # create lock for synchronization
        self.action_lock = threading.Lock()

        # Update node handle
        self.logger = logger
        if self.logger == None:
            return

        # --- initialize Moveit! variables 
        self.robot = robot
        if self.robot == None:
            return
        self.scene = self.robot.get_planning_scene_monitor()

        # --- These attributes need configuring prior to use (may be different based on project)
        self.move_group = None
        self.set_move_group(group_name=moveit_group_name)       
        self.end_effector_link = None
        self.set_ee_link() 

        # print(f"robot state: {RobotState(self.robot.get_robot_model()).get_frame_transform(self.end_effector_link)}")
        # if world_link is not None:
        #     self.WORLD_REFERENCE_LINK = world_link
        #     rospy.loginfo(f'default reference frame changed from {self.move_group.get_planning_frame()} to {world_link}')
        #     self.move_group.set_pose_reference_frame(world_link)
        # else:
        #     self.WORLD_REFERENCE_LINK = self.move_group.get_planning_frame()
        #     rospy.loginfo(f'default reference frame is {self.WORLD_REFERENCE_LINK}')

        # --- ROS Subscribers
        # TODO: check if this name can change
        # self.moveit_feedback_sub = self._nh.create_subscription(
        #     name='/move_group/feedback', 
        #     data_class=MoveGroupActionFeedback, 
        #     callback=self.cb_move_group_feedback, 
        #     queue_size=1
        # )
        self.agent_state: RobotAgentStates = RobotAgentStates.READY
        self.action_state: MoveitActionStates = MoveitActionStates.IDLE
        # self.cached_feedback: MoveGroupActionFeedback = None

        # --- ROS Publishers
        # self.scene_pub = self.logger.create_publisher('planning_scene', PlanningScene, queue_size=10)

    # # --- ROS Subscriber Callbacks
    # def cb_move_group_feedback(self, msg: MoveGroupActionFeedback) -> None:
    #     """Subscriber callback to the move_group feedback

    #     :param msg: Move group action feedback message if published
    #     :type msg: MoveGroupActionFeedback
    #     """
    #     self.cached_feedback = msg
    #     if self.agent_state == RobotAgentStates.BUSY:
    #         # --- Handle status message
    #         if msg.status.status in [GoalStatus.SUCCEEDED]:
    #             self.logger.info(f'[CustomCommander::cb_move_group_feedback][Goal Ended: {msg.status.text}]')
    #             self.agent_state = RobotAgentStates.SUCCEEDED
    #         elif msg.status.status in [GoalStatus.ABORTED]:
    #             self.logger.info(f'[CustomCommander::cb_move_group_feedback][Goal Ended: {msg.status.text}]')
    #             self.agent_state = RobotAgentStates.ABORTED

    #         # --- Handle feedback state
    #         if msg.feedback.state == 'PLANNING':
    #             self.action_state = MoveitActionStates.PLANNING
    #         elif msg.feedback.state == 'MONITOR':
    #             self.action_state = MoveitActionStates.MONITOR
    #         elif msg.feedback.state == 'IDLE':
    #             if msg.status.text == 'TIMED_OUT':
    #                 self.action_state = MoveitActionStates.ERROR
    #             else:
    #                 self.action_state = MoveitActionStates.COMPLETED 

    #     return None
    
    # --- Robot Utility Methods
    def reset(self) -> None:
        """Reset of states and variables on command
        """
        # -- set state
        self.agent_state = RobotAgentStates.READY
        self.action_state = MoveitActionStates.IDLE
        self.cached_feedback = None 

        return None

    def wait_while_busy(self) -> None:
        """Wait method while robot is busy

        :return: None
        :rtype: None
        """
        while True:
            if self.agent_state not in [RobotAgentStates.BUSY]:
                return self.agent_state
            timeit.sleep(0.05)

    # def abort_move(self, wait: bool = True) -> bool:
    #     """Aborts motion on command

    #     :param wait: Waits for completion if True, defaults to True
    #     :type wait: bool, optional
    #     :return: True on success, else False
    #     :rtype: bool
    #     """
    #     if self.is_move_group_valid():
    #         self.move_group.stop()
    #         self.move_group.clear_pose_targets()
    #         if wait:
    #             self.wait_while_busy()
    #             self.reset()
            
    #         return True
    #     else:
    #         return False
    
    # --- Robot Movement Methods
    def plan_and_execute(self) -> ExecutionStatus:
        """A common plan and execute method for the robot

        :return: True on successful planning, else False
        :rtype: bool
        """

        # --- Execute on Successful Plan
        plan_result = self.move_group.plan()
        if plan_result:
            self.logger.info(f"[CustomCommander::plan_and_execute][Executing Valid Plan...]")
            robot_trajectory = plan_result.trajectory
            outcome = self.robot.execute(robot_trajectory, controllers=[])
            return outcome
        else:
            self.logger.error(f"[CustomCommander::plan_and_execute][Plan Failed]")
            return None

    def move_to_named_pose(self, named_pose: str = "", wait: bool = True) -> None:
        """Move to a Named Pose on Command

        :param named_pose: Name of pose to move to, defaults to ""
        :type named_pose: str, optional
        :param wait: Waits for completion if True, defaults to True
        :type wait: bool, optional
        """
        self.action_lock.acquire()

        # Check if move_group is valid
        if self.is_move_group_valid():
            try:
                if self.agent_state != RobotAgentStates.READY:
                    self.logger.error(f'[CustomCommander::move_to_named_pose][MoveitAgent move_to_named_pose: not READY state]')
                    return None

                # Set to plan from current state to provided goal state
                self.move_group.set_start_state_to_current_state()
                self.move_group.set_goal_state(configuration_name=named_pose)
                self.agent_state = RobotAgentStates.BUSY    

                # --- Create Default Plan    
                outcome = self.plan_and_execute()
                
                if not wait:
                    return None
                
                if outcome == None:
                    self.logger.error(f'[CustomCommander::move_to_named_pose][Invalid Plan or Execution]')
                    return None
                
                self.logger.info(f'[CustomCommander::move_to_named_pose][Execution Status: {outcome.status}]')           
            finally:
                self.action_lock.release()
        else:
            self.logger.error(f'[CustomCommander::move_to_named_pose][Move group is invalid]')
            return None
    
    # command the robot arm to move the end effector to a position (x, y, z) optionally in a cartesian motion manner and 
    # optionally in a particular frame of reference
    def move_to_position(self, x:float=None, y:float=None, z:float=None, 
                         accept_fraction:float=0.9999, cartesian=False, 
                         reference_frame:str=None, wait=True):
        """Command the robot arm to move the end effector to a position (x, y, z) optionally 
        in a cartesian motion manner and optionally in a specified frame of reference

        :param x:The target x position, defaults to the current x position
        :type x: float, optional
        :param y: The target y position, defaults to the current y position
        :type y: float, optional
        :param z: The target z position, defaults to the current z position
        :type z: float, optional
        :param accept_fraction: the acceptable minimum fraction of the planned path towards the target or abort the command, defaults to 0.9999
        :type accept_fraction: float, optional
        :param cartesian: Use cartesian motion as the path, defaults to False
        :type cartesian: bool, optional
        :param reference_frame: The frame of reference in which positions are specified, defaults to the world/default
        :type reference_frame: str, optional
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose in the given reference frame
        :rtype: Pose
        """
        self.action_lock.acquire()
        try:
            reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
            target_pose:PoseStamped = self.pose_in_frame(self.end_effector_link, reference_frame)
            target_pose.pose.position.x = x if x is not None else target_pose.pose.position.x
            target_pose.pose.position.y = y if y is not None else target_pose.pose.position.y
            target_pose.pose.position.z = z if z is not None else target_pose.pose.position.z
            if cartesian:
                current_in_world_frame:PoseStamped = self.pose_in_frame(self.end_effector_link, self.WORLD_REFERENCE_LINK)
                target_in_world_frame:PoseStamped = self.transform_pose(target_pose, self.WORLD_REFERENCE_LINK)
                waypoints = [current_in_world_frame.pose, target_in_world_frame.pose]
                self.commander_state = CustomCommanderStates.BUSY
                (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, self.CARTE_PLANNING_STEP_SIZE, 0.00) 
                if fraction < accept_fraction:
                    rospy.logerr(f'Planning failed due to collision ({fraction})')
                    self.commander_state, self.commander_state.message = CustomCommanderStates.ABORTED, 'PLANNING_FAILED_DUE_TO_COLLISION'
                    return None 
                self.move_group.execute(plan, wait=wait)
            else:
                self.move_group.set_pose_target(target_pose)
                self.commander_state = CustomCommanderStates.BUSY
                self.move_group.go(wait=wait)
            if not wait:
                return target_pose
            self.abort_move()
            return target_pose 
        except Exception as e:
            rospy.logerr(f'System error: {e} {traceback.format_exc()}')
            self.commander_state = CustomCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()

    # # --- Robot Query Methods
    # def get_current_joint_positions(self, print: bool = False) -> dict:
    #     """Query the robot's joint positions

    #     :param print: Enable printing if True, defaults to False
    #     :type print: bool, optional
    #     :return: Dictionary of joint values
    #     :rtype: dict
    #     """
    #     # Check if move_group is valid
    #     if self.is_move_group_valid():
    #         joint_values_dict = self.robot.get_current_variable_values()
    #         if print: self.logger.info(f"[CustomCommander::get_current_joint_positions][{joint_values_dict}]")
    #         return joint_values_dict
    #     else:
    #         self.logger.error(f"[CustomCommander::get_current_joint_positions][Move group is invalid, returning empty]")
    #         return dict()
        
    def get_current_link_pose(self, link_name: str = None, print: bool = False) -> Pose:
        """Query the robot's link pose

        :param link_name: Name of link to query, defaults to None
        :type link_name: str, optional
        :param print: Debugging if True, defaults to False
        :type print: bool, optional
        :return: Pose (geometry messages)
        :rtype: Pose
        """
        # Check if move_group is valid
        if self.is_move_group_valid() and self.is_ee_link_valid():
            # Default to end-effector link if not defined
            if link_name is None:
                link_name = self.end_effector_link
            self.logger.info(f"Checking for Link: {link_name}")

            # Get the current pose from the robot state
            robot_state = RobotState(self.robot.get_robot_model())
            robot_state.update()

            current_pose = robot_state.get_pose(link_name)
            if print: self.logger.info(f"[CustomCommander::get_current_link_pose][current_pose: {current_pose}]")
            return current_pose
        else:
            self.logger.error(f"[CustomCommander::get_current_link_pose][Move group or end-effector link invalid]")
            return Pose()
        
    # # --- General Get Methods
    # def get_latest_feedback(self) -> MoveGroupActionFeedback:
    #     """Gets the latest feedback message cached from callback

    #     :return: Move group cached feedback message
    #     :rtype: MoveGroupActionFeedback
    #     """
    #     return self.cached_feedback
    
    def get_agent_state(self, print: bool = False) -> RobotAgentStates:
        """Gets the agent's state

        :param print: Prints the agent state name if True, defaults to False
        :type print: bool, optional
        :return: Robot agent state
        :rtype: RobotAgentStates
        """
        if print: self.logger.info(f"[CustomCommander::get_agent_state][name: {self.agent_state.name}]")
        return self.action_state

    # --- General Set Methods
    def set_ee_link(self, link_name=None) -> bool:
        """Sets the end-effector (ee) link through move_group

        :return: True if successful, else False
        :rtype: bool
        """
        # check if move_group has been set
        if self.is_move_group_valid():
            # Get the joint model group based on the move group name
            joint_model_group = self.robot.get_robot_model().get_joint_model_group(
                self.move_group.planning_group_name
            )
            link_list = list(joint_model_group.link_model_names)
            if link_name == None or link_name not in link_list:
                # Extract the last object in the list as the end-effector link
                self.end_effector_link = link_list[-1]
                self.logger.info(f"[CustomCommander::set_ee_link][Successfully set ee link: {self.end_effector_link}]")
            else:
                # Update based on provided link name as it is in link list
                self.end_effector_link = link_name
        else:
            self.logger.error(f"[CustomCommander::set_ee_link][move_group not valid, cannot set ee link]")
            return False

    def set_move_group(self, group_name: str = "") -> str:
        """Sets the move group attribute on provided group name

        :param group: name of move_group, defaults to ""
        :type group: str, optional
        :return: True if successful, otherwise False
        :rtype: bool
        """
        if group_name == "" or group_name == None:
            self.logger.error(f"[CustomCommander::set_move_group][Invalid group name given: {group_name}]")
            return None
        
        if self.move_group == None:
            # If not set, create a move group commander with group name provided
            self.move_group = self.robot.get_planning_component(group_name)
            self.logger.info(f"[CustomCommander::set_move_group][Successfully set move_group: {group_name}]")
            return True
        else:
            self.logger.error(f"[CustomCommander::set_move_group][move_group already set]")
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