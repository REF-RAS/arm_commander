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
import timeit, tf2_ros, tf_transformations
import spatialmath as sm
# from tf_transformations import euler_from_quaternion
from rclpy.logging import get_logger

# --- Import moveit_py bindings to moveit core (Official)
from moveit.core.robot_state import RobotState
from moveit.core.controller_manager import ExecutionStatus
from moveit.planning import MoveItPy

# --- Import ROS Messages
from moveit_msgs.msg import RobotTrajectory, PlanningScene, ObjectColor 
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from std_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped

# Import State Definitions
from arm_commander_py.states import CommanderStates 

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
        # NOTE: new variable to capture execution status
        self.execution_status: ExecutionStatus = None

        # --- Define workspace walls
        self.wall_name_list = []

        # --- Transform Publishers
        # NOTE: requires node handle passed into class
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_pub = tf2_ros.TransformBroadcaster(node=nh)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node=nh)
        self.nh = nh

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
        self.logger.warn(f"[ArmCommander::_pub_workspace_color][Not Yet Implemented]")

    def _pub_transform_all_objects(self) -> None:
        """TODO: publish transforms of all objects in scene
        """
        self.logger.warn(f"[ArmCommander::_pub_transform_all_objects][Not Yet Implemented]")

    def _pub_transform_object(self, name, pose) -> None:
        """TODO: publish transform of a specific named object

        :param name: _description_
        :type name: _type_
        :param pose: _description_
        :type pose: _type_
        """
        self.logger.warn(f"[ArmCommander::_pub_transform_object][Not Yet Implemented]")

    def _cb_move_group_result(self) -> None:
        """TODO: alternative direct API version of this
        """
        self.logger.warn(f"[ArmCommander::_cb_move_group_result][Not Yet Implemented]")

    def _cb_trajectory_execute_result(self) -> None:
        """TODO: alternative direct API version of this
        """
        self.logger.warn(f"[ArmCommander::_cb_trajectory_execute_result][Not Yet Implemented]")

    def _cb_handle_result_status(self) -> None:
        """TODO: alternative direct API version of this

        :return: _description_
        :rtype: _type_
        """
        self.logger.warn(f"[ArmCommander::_cb_handle_result_status][Not Yet Implemented]")

    def cb_execute(self, outcome) -> None:
        """Callback to Trajectory Execution Manager execution call

        :param outcome: Status of execution
        :type outcome: ExecutionStatus
        """
        self.execution_status = outcome
        self.logger.info(f"[ArmCommander::cb_execute][Execution Status: {outcome.status}]")

    # --- Utility Methods
    def spin(self) -> None:
        """Spins up the ROS2 Node
        """
        rclpy.spin(self.nh)

    def shutdown(self) -> None:
        """Shutdown command for moveit_py

        :return: None
        :rtype: None
        """
        if self.robot: self.robot.shutdown()

    def reset_state(self) -> None:
        """Reset of states and variables on command

        :return: None
        :rtype: None
        """
        # -- set state
        self.commander_state = CommanderStates.READY
        self.cached_result = None 

        return None

    def wait_for_busy_end(self) -> CommanderStates:
        """Wait method while robot is busy

        :return: The final state of the commander
        :rtype: CommanderStates
        """
        while True:
            if self.commander_state not in [CommanderStates.BUSY]:
                return self.commander_state
            timeit.sleep(0.05)

    def abort_move(self, wait: bool = True) -> bool:
        """Aborts motion on command

        :param wait: Waits for completion if True, defaults to True
        :type wait: bool, optional
        :return: True on success, else False
        :rtype: bool
        """
        if self.is_move_group_valid():
            # Get the Trajectory Execution Manager
            tem = self.robot.get_trajactory_execution_manager()

            # Request execution stop
            tem.stop_execution()

            self.logger.info(f"[ArmCommander::abort_move][Execution Stopped]")
            
            # if wait:
            #     self.wait_while_busy()
            self.reset_state()
            
            return True
        else:
            return False
    
    # --- Robot Movement Methods
    def plan_and_execute(self, wait: bool = False) -> None:
        """A common plan and execute method for the robot

        :param wait: Set True to wait for execution, defaults to False
        :type wait: bool, optional
        :return: None (outcome in callback -> see ArmCommander::cb_plan_execute_status())
        :rtype: None
        """

        # --- Execute on Successful Plan
        plan_result = self.move_group.plan()
        if plan_result:
            self.logger.info(f"[ArmCommander::plan_and_execute][Executing Valid Plan...]")
            robot_trajectory = plan_result.trajectory

            # Check if there are controllers to execute
            tem = self.robot.get_trajactory_execution_manager()
            if not tem.ensure_active_controllers_for_group(self.GROUP_NAME):
                self.logger.error(f"[ArmCommander::plan_and_execute][No active controllers]")
                return None
            
            # Execute the trajectory with no wait
            rt = RobotTrajectory()
            rt = robot_trajectory.get_robot_trajectory_msg()
            # Pass robot trajectory as a moveit RobotTrajectory object with controller list
            tem.push(rt, '')
            # Retrieve status from callback
            tem.execute(callback=self.cb_execute)
            if wait:  
                self.logger.info(f"[ArmCommander::plan_and_execute][Execution in Progress; Waiting...]")
                self.commander_state = CommanderStates.BUSY
                tem.wait_for_execution()
            else:
                self.logger.info(f"[ArmCommander::plan_and_execute][Execution in Progress; Skipping Wait]")
            
            self.logger.info(f"[ArmCommander::plan_and_execute][Execution Request Complete]")
            return None
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

                # --- Create Default Plan and Execute with/without Wait  
                self.plan_and_execute(wait=wait)
                
                self.logger.info(f'[ArmCommander::move_to_named_pose][Execution Requested]')           
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
        # try:
        reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
        target_pose:PoseStamped = self.pose_in_frame(self.end_effector_link, reference_frame)
        target_pose.pose.position.x = x if x is not None else target_pose.pose.position.x
        target_pose.pose.position.y = y if y is not None else target_pose.pose.position.y
        target_pose.pose.position.z = z if z is not None else target_pose.pose.position.z

        self.move_group.set_start_state_to_current_state()
        if cartesian:
            current_in_world_frame:PoseStamped = self.pose_in_frame(self.end_effector_link, self.WORLD_REFERENCE_LINK)
            # target_in_world_frame:PoseStamped = self.transform_pose(target_pose, self.WORLD_REFERENCE_LINK)
            # waypoints = [current_in_world_frame.pose, target_in_world_frame.pose]
            self.commander_state = CommanderStates.BUSY

            robot_state = RobotState(self.robot.get_robot_model())
            # robot_state.compute_cartesian_path()
            # plan_result = self.move_group.set_path_constraints(target_pose)
            # self.logger.info(f"MOVE TO POSITION Waypoints: {waypoints}")
            # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, self.CARTE_PLANNING_STEP_SIZE, 0.00) 
            # if fraction < accept_fraction:
            #     rospy.logerr(f'Planning failed due to collision ({fraction})')
            #     self.commander_state, self.commander_state.message = CommanderStates.ABORTED, 'PLANNING_FAILED_DUE_TO_COLLISION'
            #     return None 
            # self.move_group.execute(plan, wait=wait)
        else:
            self.commander_state = CommanderStates.BUSY
            self.logger.info(f"[ArmCommander::move_to_position][Target: {target_pose}]")
            # NOTE: both pose stamped message and pose link (end-effector) is needed to set goal state
            self.move_group.set_goal_state(pose_stamped_msg=target_pose, pose_link=self.end_effector_link)

            # --- Create Default Plan and Execute with/without Wait  
            self.plan_and_execute(wait=wait)

        if not wait:
            return target_pose
        
        self.abort_move()
        self.action_lock.release()

        return target_pose 
        # except Exception as e:
        #     self.logger.error(f'[ArmCommander::move_to_position][System error: {e}]')
        #     self.commander_state = CommanderStates.ABORTED
        #     self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        # finally:
        self.action_lock.release()

    # --- Robot Query Methods
    def current_joint_positions(self, print=False) -> dict:
        """ Get the positions of all the joints

        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: Pairs of (joint_name, position) in a dictionary
        :rtype: dict
        """
        self.logger.warn(f"[ArmCommander::current_joint_positions][Not Yet Implemented]")
        # TODO: update this method for moveit_py
        joint_values_dict = dict() #self.robot.get_current_variable_values()
        if print: self.logger.info(joint_values_dict)
        return joint_values_dict
    
    def current_joint_positons_as_list(self, print=False) -> list:
        """ Get the positions of all the joints

        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: Lists of joint positions
        :rtype: list
        """
        self.logger.warn(f"[ArmCommander::current_joint_positions_as_list][Not Yet Implemented]")
        # TODO: update this method for moveit_py
        joint_values = list() #self.move_group.get_current_joint_values()
        if print: self.logger.info(joint_values)
        return joint_values   

    def pose_in_frame_as_xyzq(self, link_name:str=None, reference_frame:str=None, ros_time:rclpy.time.Time=None, print=False) -> list:
        """ Get the pose of a link as a list of 7 floats (xyzq)

        :param link_name: The name of the link to be queried, defaults to the end-effector
        :type link_name: str, optional
        :param reference_frame: The name of the frame of reference, defaults to the world/default
        :type reference_frame: str, optional
        :param ros_time: The time when the pose is queried, defaults to current time
        :type ros_time: rclpy.time.Time, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :raises ValueError: when the names of the link_name or reference_frame is invalid
        :return: The pose in the format of a list of 7 floats (xyzq)
        :rtype: list
        """
        link_name = self.end_effector_link if link_name is None else link_name
        ros_time = rclpy.time.Time() if ros_time is None else ros_time
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame    
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame=reference_frame,
                source_frame=link_name,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # NOTE: quaternion in x, y, z, w order
            trans_list: list = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ]

            if print: 
                self.logger.info(f'[ArmCommander::pose_in_frame_as_xyzq][pose of {link_name} from {reference_frame}: {trans_list}]')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            raise ValueError(f'Invalid parameter or TF error: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.logger.error(f"[ArmCommander::pose_in_frame_as_xyzq][Extrapolation Error: {e}]")
            raise
        
        return trans_list
    
    def pose_in_frame_as_xyzrpy(self, link_name:str=None, reference_frame:str=None, ros_time:rclpy.time.Time=None, print=False) -> list:
        """ Get the pose of a link as a list of 6 floats (xyzrpy)

        :param link_name: The name of the link to be queried, defaults to the end-effector
        :type link_name: str, optional
        :param reference_frame: The name of the frame of reference, defaults to the world/default
        :type reference_frame: str, optional
        :param ros_time: The time when the pose is queried, defaults to current time
        :type ros_time: rclpy.time.Time, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :raises ValueError: when the names of the link_name or reference_frame is invalid
        :return: The pose in the format of a list of 6 floats (xyzrpy)
        :rtype: list
        """
        # Initial setup
        link_name = self.end_effector_link if link_name is None else link_name
        ros_time = rclpy.time.Time() if ros_time is None else ros_time
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame 

        # Request 7 float list of pose
        pose = self.pose_in_frame_as_xyzq(link_name, reference_frame, ros_time)

        # Extract and extend list with rpy information
        xyzrpy = pose[:3]
        xyzrpy.extend(tf_transformations.euler_from_quaternion(pose[3:]))
        if print: 
            self.logger.info(f'[ArmCommander::pose_in_frame_as_xyzrpy][pose of {link_name} from {reference_frame}: {xyzrpy}]')
        return xyzrpy
    
    def pose_in_frame(self, link_name:str=None, reference_frame:str=None, ros_time:rclpy.time.Time=None) -> PoseStamped:
        """ Get the pose of a link as a PoseStamped

        :param link_name: The name of the link to be queried, defaults to the end-effector
        :type link_name: str, optional
        :param reference_frame: The name of the frame of reference, defaults to the world/default
        :type reference_frame: str, optional
        :param ros_time: The time when the pose is queried, defaults to current time
        :type ros_time: rclpy.tim.Time, optional
        :raises ValueError: when the names of the link_name or reference_frame is invalid
        :return: The pose as a PoseStamped
        :rtype: PoseStamped
        """
        # Initial setup
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame        
        link_name = self.end_effector_link if link_name is None else link_name
        ros_time = rclpy.time.Time() if ros_time is None else ros_time

        # Request 7 float list of pose
        xyzq = self.pose_in_frame_as_xyzq(link_name, reference_frame, ros_time)

        # Construct PoseStamped Message
        # NOTE: time stamp populated from node handle
        pose_stamped_out = PoseStamped()
        pose_stamped_out.header.frame_id = reference_frame
        pose_stamped_out.header.stamp = self.nh.get_clock().now().to_msg()

        if len(xyzq) != 7:
            self.logger.error(f"[ArmCommander::pose_in_frame][size of xyzq is invalid: {len(xyzq)}]")
            pose_stamped_out.pose = Pose()
        else:
            pose_stamped_out.pose.position.x = xyzq[0]
            pose_stamped_out.pose.position.y = xyzq[1]
            pose_stamped_out.pose.position.z = xyzq[2]
            pose_stamped_out.pose.orientation.x = xyzq[3]
            pose_stamped_out.pose.orientation.y = xyzq[4]
            pose_stamped_out.pose.orientation.z = xyzq[5]
            pose_stamped_out.pose.orientation.w = xyzq[6]

        return pose_stamped_out
        
    def pose_of_robot_link(self, robot_link_name: str = None, print: bool = False) -> PoseStamped:
        """Query the robot's link pose as a stamped message

        :param link_name: Name of link to query, defaults to None
        :type link_name: str, optional
        :param print: Debugging if True, defaults to False
        :type print: bool, optional
        :return: Pose (geometry messages) in a pose stamped message
        :rtype: PoseStamped
        """
        # Request pose of link from robot
        pose = self._get_pose(link_name=robot_link_name)
        
        # Convert to a posestamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.WORLD_REFERENCE_LINK
        pose_stamped.header.stamp = self.nh.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped
    
    def rpy_of_robot_link(self, robot_link_name: str = None, print: bool = False) -> list:
        """ Get the orientation of a link as a list of euler angles (rpy) in the default move_group planning frame

        :param robot_link_name: The name of the link to be queried, defaults to the end-effector
        :type robot_link_name: str, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of a link as a list of euler angles (rpy) 
        :rtype: list
        """
        if robot_link_name is None:
            robot_link_name = self.end_effector_link

        # Retrieve pose stamped message of requested link
        pose = self._get_pose(link_name=robot_link_name)

        # Convert to RPY
        # NOTE: x,y,z,w quaternion order assumed
        rpy = tf_transformations.euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
        )
        if print: self.logger.info(f"[ArmCommander::rpy_of_robot_link][{rpy}]")
        return rpy
        
    def xyzrpy_of_robot_link(self, robot_link_name: str = None, print: bool = False) -> list:
        """ Get the pose of a link as a list of 6 floats (xyzrpy) in the default move_group planning frame

        :param robot_link_name: The name of the link to be queried, defaults to the end-effector
        :type robot_link_name: str, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of a link as a list of 6 floats (xyzrpy)
        :rtype: list
        """
        if robot_link_name is None:
            robot_link_name = self.end_effector_link
            
        # Retrieve pose of requested link
        pose = self._get_pose(link_name=robot_link_name)

        # Retrieve pose as rpy
        rpy = self.rpy_of_robot_link(robot_link_name=robot_link_name)

        # Construct xyzrpy list
        xyzrpy = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]
        xyzrpy.extend(rpy)
        if print: self.logger.info(f"[ArmCommander::xyzrpy_of_robot_link][{xyzrpy}]")
        return xyzrpy
    
    def transform_pose(self, the_pose: PoseStamped, to_frame: str) -> PoseStamped:
        """ Transorm a pose from the current frame of reference to another frame of reference

        :param the_pose: The pose to be transformed
        :type the_pose: PoseStamped
        :param to_frame: The target frame of reference
        :type to_frame: str
        :return: The transformed pose
        :rtype: PoseStamped
        """
        try:
            transformed_pose = self.tf_buffer.transform(
                object_stamped=the_pose,
                target_frame=to_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.ExtrapolationException as e:
            self.logger.error(f"[ArmCommander::transform_pose][Extrapolation Error: {e}]")
            raise
    
        return transformed_pose
    
    # set the joint positions (as a list) of a named pose
    def add_named_pose(self, name: str, joint_values: list):
        """ Add a named pose specified in joint positions to the commander

        :param name: The name of the pose to be defined
        :type name: str
        :param joint_values: a list of joint values
        :type joint_values: list
        """
        self.logger.warn(f"[ArmCommander::add_named_pose][Not Yet Implemented]")
        # self.move_group.remember_joint_values(name, values=joint_values)
    
    # remove the named pose from the commander
    def forget_named_pose(self, name: str):
        """Remove the named pose from the commander

        :param name: The name of the pose to be defined
        :type name: str
        """
        self.logger.warn(f"[ArmCommander::add_named_pose][Not Yet Implemented]")
        # self.move_group.forget_joint_values(name)    

    # --- General Get Methods
    def _get_pose(self, link_name: str = None, print: bool = False) -> Pose():
        """Internal method to get the pose of the robot

        :param link_name: Name of link to query, defaults to None
        :type link_name: str, optional
        :param print: debugging if True
        :type print: bool, optional 
        :return: Pose (geometry messages)
        :rtype: Pose
        """
        if self.is_move_group_valid() and self.is_ee_link_valid():
            # Default to end-effector link if not defined
            if link_name is None:
                link_name = self.end_effector_link

            # Get the current pose from the robot state
            robot_state = RobotState(self.robot.get_robot_model())
            robot_state.update()

            pose = robot_state.get_pose(link_name)
            if print: self.logger.info(f"[ArmCommander::_get_pose][Link {link_name} has current_pose: {pose}]")
        else:
            self.logger.error(f"[ArmCommander::_get_pose][Move group or end-effector link invalid]")
            pose = Pose()

        return pose
    
    def info(self, print: bool = False) -> str:
        """TODO: Essential information for display

        :param print: Sends the information to the output, defaults to False
        :type print: bool, optional
        :return: Formatted information of the commander
        :rtype: str
        """
        nyi = "Not Yet Implemented"
        string_list = [
        f'> group name: {self.GROUP_NAME}',
        f'> planning time: {nyi}',
        f'> pose reference frame: {self.WORLD_REFERENCE_LINK}',   
        f'> end-effector:\t\nlinks: {self.end_effector_link}',
        f'> pose: {self.pose_of_robot_link().pose}',
        f'> roll, pitch, yaw: {nyi}',
        f'> goal tolerance (joint, position, orientation): {nyi}'
        ]
        output = '\n'.join(string_list)
        if print:
            self.logger.info(output)
        return output
    
    def get_latest_moveit_feedback(self) -> None:
        """Get the latest feedback resulting from the last command

        :return: The latest action feedback of the last call to the underlying move commander
        :rtype: None
        """
        self.logger.warn(f"[ArmCommander::get_latest_moveit_feedback][Not Yet Implemented]")
        return self.cached_result
    
    def get_commander_state(self, print=False) -> CommanderStates:
        """Get the current state of this general commander

        :param print: Sends the state to the output as well, defaults to False
        :type print: bool, optional
        :return: The current state of this general commander
        :rtype: CommanderStates
        """
        if print:
            self.logger.info(self.commander_state.name)
        return self.commander_state
    
    def get_error_code(self, print=False) -> None:
        """Get the error code of resulting from the last command

        :param print: Sends the error code to the output as well, defaults to False
        :type print: bool, optional
        :return: The error code of the last command
        :rtype: None
        """
        self.logger.warn(f"[ArmCommander::get_error_code][Not Yet Implemented]")
        if self.cached_result is None:
            return None
        if print:
            # TODO: implement this as a version of MoveitErrorCodes
            pass

        return None #self.cached_result.result.error_code 
    
    def get_goal_tolerance(self) -> list:
        """ Get the current goal tolerance

        :return: A list of current joint, position and orientation goal tolerances
        :rtype: list
        """
        self.logger.warn(f"[ArmCommander::get_goal_tolerance][Not Yet Implemented]")
        return None #self.move_group.get_goal_tolerance()

    # --- General Set Methods
    def set_max_cartesian_speed(self, max_speed: float = None):
        """Set the maximum speed of the end-effector motion

        :param max_speed: The maximum speed in meters per seconds, defaults to the system default speed
        :type max_speed: float, optional
        """
        self.logger.warn(f"[ArmCommander::set_max_cartesian_speed][Not Yet Implemented]")

        # if max_speed is None:
        #     self.move_group.clear_max_cartesian_link_speed()
        # else:
        #     self.move_group.limit_max_cartesian_link_speed(max_speed, self.end_effector_link)

    
    # set the maximum planning time (in seconds)
    def set_planning_time(self, planning_time: int):
        """Set the maximum planning time

        :param planning_time: The maximum planning time in seconds
        :type planning_time: int
        """
        self.logger.warn(f"[ArmCommander::set_planning_time][Not Yet Implemented]")
        # self.move_group.set_planning_time(planning_time)
    
    # set the goal tolerance (a list of joint, position and orientation goal tolerances)
    def set_goal_tolerance(self, tol_list: list): 
        """Set the goal tolerance 

        :param tol_list: set the goal tolerance as a list of joint, position and orientation goal tolerances
        :type tol_list: list
        """
        self.logger.warn(f"[ArmCommander::set_goal_tolerance][Not Yet Implemented]")
        # self.move_group.set_goal_tolerance(tol_list) 

    def set_ee_link(self, link_name = None) -> bool:
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