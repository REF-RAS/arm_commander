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
import timeit, tf2_ros
from rclpy.logging import get_logger

# --- Import moveit_py bindings to moveit core (Official)
from moveit.core.robot_state import RobotState
from moveit.core.controller_manager import ExecutionStatus
from moveit.planning import MoveItPy

# --- Import ROS Messages
# from moveit_msgs.msg import MoveGroupActionFeedback, PlanningScene, ObjectColor 
# from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from std_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped

# Import State Definitions
from ref_moveit_interface.states import CommanderStates 

class ArmCommander():
    def __init__(self, 
                nh = None,
                moveit_group_name: str = None, 
                world_link: str = None) -> None:
        
        # --- Define the logging interface to use
        self.logger = get_logger("arm_commander")        
        
        # --- Create lock for synchronization
        self.action_lock = threading.Lock()

        # --- Define constants (should import from config)
        self.CARTE_PLANNING_STEP_SIZE = 0.01  # meters
        self.GROUP_NAME = moveit_group_name

        # --- initialize Moveit! variables 
        # Equivalent moveit_commander.RobotCommander()
        # A moveit_py bind to moveit_cpp
        self.robot = MoveItPy(node_name="arm_commander")
        # Equivalent moveit_commander.PlanningSceneInterface
        self.scene = self.robot.get_planning_scene_monitor()
        # Equivalent moveit_commander.MoveGroupCommander
        self.move_group = self.robot.get_planning_component(moveit_group_name)       
        # initialize the world_link as the default planning frame
        self.end_effector_link = None
        self.set_ee_link() 
        self.WORLD_REFERENCE_LINK = None
        if world_link is not None:
            self.WORLD_REFERENCE_LINK = world_link
            with self.scene.read_write() as scene:
                self.logger.info(f'[ArmCommander::init][default reference frame changed from {scene.planning_frame} to {world_link}')
                self.logger.warn(f"[ArmCommander::init][Setting Not Available]")
        else:
            with self.scene.read_only() as scene:
                self.WORLD_REFERENCE_LINK = scene.planning_frame
            self.logger.info(f'[ArmCommander::init][default reference frame is {self.WORLD_REFERENCE_LINK}]')

        # --- ROS Subscribers
        # NOTE: feedback subscriber not required due to direct API calls
            
        # --- ROS Publishers
        # NOTE: scene publisher may not be required due to direct API calls
            
        # --- Internal States
        self.commander_state: CommanderStates = CommanderStates.READY
        # NOTE: may not be needed due to direct API calls
        self.cached_result = None

        # --- Define workspace walls
        self.wall_name_list = []

        # --- Transform Publishers
        # NOTE: requires node handle passed into class
        self.tf_pub = tf2_ros.TransformBroadcaster(node=nh)
        self.tf_listener = tf2_ros.TransformListener(tf2_ros.Buffer(), node=nh)

        # --- Constraints
        # NOTE: may have an alternative implementation 
        self.the_constraints = None

        # --- Set an internal timeout
        # NOTE: may have an alternative implementation
        self.timer = None
    
    # --- Internal Methods
    def _cb_timer(self, event) -> None:
        """TODO: implement timer for waking up IDLE commander

        :param event: _description_
        :type event: _type_
        """
        self._pub_workspace_color()
        self._pub_transform_all_objects()

    def _pub_workspace_color(self) -> None:
        """TODO: publish colours of workspace (wall) objects
        """
        pass

    def _pub_transform_all_objects(self) -> None:
        """TODO: publish transforms of all objects in scene
        """
        pass

    def _pub_transform_object(self, name, pose) -> None:
        """TODO: publish transform of a specific named object

        :param name: _description_
        :type name: _type_
        :param pose: _description_
        :type pose: _type_
        """
        pass

    def _cb_move_group_result(self) -> None:
        """TODO: alternative direct API version of this
        """
        pass

    def _cb_trajectory_execute_result(self) -> None:
        """TODO: alternative direct API version of this
        """
        pass

    def _cb_handle_result_status(self) -> None:
        """TODO: alternative direct API version of this

        :return: _description_
        :rtype: _type_
        """
        pass

    # --- Utility Methods
    def info(self, print: bool = False) -> str:
        """TODO: Essential information for display

        :param print: Sends the information to the output, defaults to False
        :type print: bool, optional
        :return: Formatted information of the commander
        :rtype: str
        """
        string_list = [
        f'group name: {self.GROUP_NAME}',
        f'planning time: {None}'
        f'pose reference frame: {None}',   
        f'end-effector:\nlinks: {self.end_effector_link}',
        f'pose: {None}',
        f'roll, pitch, yaw: {None}',
        f'goal tolerance (joint, position, orientation): {None}'
        ]
        output = '\n'.join(string_list)
        if print:
            self.logger.info(output)
        return output

    def shutdown(self) -> None:
        """Shutdown command for moveit_py
        """
        if self.robot: self.robot.shutdown()

    def reset(self) -> None:
        """Reset of states and variables on command
        """
        # -- set state
        self.commander_state = CommanderStates.READY
        self.cached_result = None 

        return None

    def wait_while_busy(self) -> None:
        """Wait method while robot is busy

        :return: None
        :rtype: None
        """
        while True:
            if self.commander_state not in [CommanderStates.BUSY]:
                return self.commander_state
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
    def plan_and_execute(self, wait: bool = False) -> ExecutionStatus:
        """A common plan and execute method for the robot

        :param wait: Set True to wait for execution, defaults to False
        :type wait: bool, optional
        :return: outcome with status on success, else None
        :rtype: ExecutionStatus
        """

        # --- Execute on Successful Plan
        plan_result = self.move_group.plan()
        if plan_result:
            self.logger.info(f"[ArmCommander::plan_and_execute][Executing Valid Plan...]")
            robot_trajectory = plan_result.trajectory
            if wait:
                outcome = self.robot.execute(robot_trajectory, controllers=[])
                self.robot.get_trajactory_execution_manager().wait_for_execution()
                self.logger.info(f'[ArmCommander::plan_and_execute][Completed Execution with Wait]')
            else:
                outcome = self.robot.execute(robot_trajectory, controllers=[])
            return outcome
        else:
            self.logger.error(f"[ArmCommander::plan_and_execute][Plan Failed]")
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
                if self.commander_state != CommanderStates.READY:
                    self.logger.error(f'[ArmCommander::move_to_named_pose][MoveitAgent move_to_named_pose: not READY state]')
                    return None

                # Set to plan from current state to provided goal state
                self.move_group.set_start_state_to_current_state()
                self.move_group.set_goal_state(configuration_name=named_pose)
                self.commander_state = CommanderStates.BUSY    

                # --- Create Default Plan    
                outcome = self.plan_and_execute(wait=wait)
                
                if not wait:
                    self.logger.info(f"[ArmCommander::move_to_named_pose][Skipping with No Wait]")
                    return None
                
                if outcome == None:
                    self.logger.error(f'[ArmCommander::move_to_named_pose][Invalid Plan or Execution]')
                    return None
                
                self.logger.info(f'[ArmCommander::move_to_named_pose][Execution Status: {outcome.status}]')           
            finally:
                self.action_lock.release()
        else:
            self.logger.error(f'[ArmCommander::move_to_named_pose][Move group is invalid]')
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
                self.commander_state = CommanderStates.BUSY
                (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, self.CARTE_PLANNING_STEP_SIZE, 0.00) 
                if fraction < accept_fraction:
                    rospy.logerr(f'Planning failed due to collision ({fraction})')
                    self.commander_state, self.commander_state.message = CommanderStates.ABORTED, 'PLANNING_FAILED_DUE_TO_COLLISION'
                    return None 
                self.move_group.execute(plan, wait=wait)
            else:
                self.move_group.set_pose_target(target_pose)
                self.commander_state = CommanderStates.BUSY
                self.move_group.go(wait=wait)
            if not wait:
                return target_pose
            self.abort_move()
            return target_pose 
        except Exception as e:
            rospy.logerr(f'System error: {e} {traceback.format_exc()}')
            self.commander_state = CommanderStates.ABORTED
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
    #         if print: self.logger.info(f"[ArmCommander::get_current_joint_positions][{joint_values_dict}]")
    #         return joint_values_dict
    #     else:
    #         self.logger.error(f"[ArmCommander::get_current_joint_positions][Move group is invalid, returning empty]")
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
            if print: self.logger.info(f"[ArmCommander::get_current_link_pose][current_pose: {current_pose}]")
            return current_pose
        else:
            self.logger.error(f"[ArmCommander::get_current_link_pose][Move group or end-effector link invalid]")
            return Pose()
        
    # # --- General Get Methods
    # def get_latest_feedback(self) -> MoveGroupActionFeedback:
    #     """Gets the latest feedback message cached from callback

    #     :return: Move group cached feedback message
    #     :rtype: MoveGroupActionFeedback
    #     """
    #     return self.cached_result
    
    def get_commander_state(self, print: bool = False) -> CommanderStates:
        """Gets the agent's state

        :param print: Prints the agent state name if True, defaults to False
        :type print: bool, optional
        :return: Robot agent state
        :rtype: CommanderStates
        """
        if print: self.logger.info(f"[ArmCommander::get_commander_state][name: {self.commander_state.name}]")
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
                self.logger.info(f"[ArmCommander::set_ee_link][Successfully set ee link: {self.end_effector_link}]")
            else:
                # Update based on provided link name as it is in link list
                self.end_effector_link = link_name
        else:
            self.logger.error(f"[ArmCommander::set_ee_link][move_group not valid, cannot set ee link]")
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