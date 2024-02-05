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
import sys, copy, threading, time, signal, math, rclpy, traceback
import timeit, tf2_ros, tf_transformations
# ROS Python Imports
from rclpy.logging import get_logger
from rclpy.qos import (
    QoSDurabilityPolicy, 
    QoSHistoryPolicy, 
    QoSProfile, 
    QoSReliabilityPolicy
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# --- Import ROS Messages
# Moveit! Messages
from moveit_msgs.msg import (
    PlanningScene, 
    CollisionObject,
    AttachedCollisionObject,
    Constraints,
    OrientationConstraint,
    JointConstraint,
    PositionConstraint,
    MoveItErrorCodes,
)
# Moveit! Actions
from moveit_msgs.action import (
    MoveGroup, 
    ExecuteTrajectory
)
# Moveit! Services
from moveit_msgs.srv import (
    GetCartesianPath, 
    GetMotionPlan, 
    GetPlanningScene, 
    ApplyPlanningScene
)
# Standard Messages and Actions
from std_msgs.msg import Float64
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory

# Import State Definitions
from arm_commander.states import (
    CommanderStates, 
    ConstraintOptions
)

class ArmCommander():
    def __init__(self, 
                nh = None,
                moveit_group_name: str = None, 
                world_link: str = None) -> None:
        
        # --- Initialize ArmCommander variables
        # Define the logging interface to use
        self._logger = get_logger("arm_commander")   
        self._nh = nh     
        
        # Create lock for synchronization
        self._action_lock = threading.Lock()
        self._future = threading.Event()
        self._cb_execute_goal_handle = None

        # Define constants (should import from config)
        self.CARTE_PLANNING_STEP_SIZE = 0.0001  # meters
        self._move_group_name = moveit_group_name
 
        # Initialize the ee and world_link
        self._end_effector_link = "tool0"
        self._world_reference_link = world_link
        self._move_group = True
        self._wait_for_busy_end_rate = self._nh.create_rate(100) #hz

        # Internal States
        self._commander_state: CommanderStates = CommanderStates.READY
        self._cached_result = None

        # Define workspace walls
        self._wall_name_list = []

        # Transform Publishers
        # NOTE: requires node handle passed into class
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_pub = tf2_ros.TransformBroadcaster(node=self._nh)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node=self._nh)

        # Constraints
        # NOTE: used within the default move group goal (see below)
        self._the_constraints = Constraints()
        self._the_constraints.name = 'commander'
        # Defining the Default Move Goal to Populate
        self._move_action_goal = self._move_default_goal(
            frame_id = self._world_reference_link,
            group_name = self._move_group_name,
            end_effector = self._end_effector_link
        )

        # Set an internal timeout
        # NOTE: This works as intended but commented out for now (awaiting more work)
        self._timer = self._nh.create_timer(0.05, self._cb_timer)

        # Named Pose Handling
        self._named_poses: dict = None

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        # Joint State Variables
        self._joint_state = None
        self._joint_state_mutex = threading.Lock()
        self._joint_state_ready = False

        # --- ROS QoS Profiles
        self._qos_volatile_besteffort_last_depth1 = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._qos_volatile_reliable_last_depth5 = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- ROS Subscribers
        self._nh.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self._cb_joint_state,
            qos_profile=self._qos_volatile_besteffort_last_depth1,
            callback_group=self._callback_group,
        )
        # self._nh.create_subscription(
        #     msg_type=ExecuteTrajectory.Result,
        #     topic="execute_trajectory/result",
        #     callback=self._cb_trajectory_execute_result,
        #     qos_profile=self._qos_volatile_besteffort_last_depth1,
        #     callback_group=self._callback_group
        # )
            
        # --- ROS Publishers
        self._collision_object_pub = self._nh.create_publisher(
            CollisionObject,
            "/collision_object",
            10
        )
        self._attached_collision_object_pub = self._nh.create_publisher(
            AttachedCollisionObject,
            "/attached_collision_object",
            10
        )
        self._planning_scene_pub = self._nh.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        # --- ROS Actions
        self._move_action_client = ActionClient(
            node=self._nh,
            action_type=MoveGroup,
            action_name="move_action",
            callback_group=self._callback_group
        )
        # NOTE: the action name may differ so need a config method for this
        self._follow_jt_action_client = ActionClient(
            node=self._nh,
            action_type=FollowJointTrajectory,
            action_name="joint_trajectory_controller/follow_joint_trajectory",
            callback_group=self._callback_group
        )

        # --- ROS Services
        self._plan_cartesian_path_srv = self._nh.create_client(
            srv_type=GetCartesianPath,
            srv_name="compute_cartesian_path",
            callback_group=self._callback_group
        )
        self._plan_kinematic_path_srv = self._nh.create_client(
            srv_type=GetMotionPlan,
            srv_name="plan_kinematic_path",
            callback_group=self._callback_group
        )
        self._get_planning_scene_srv = self._nh.create_client(
            srv_type=GetPlanningScene,
            srv_name="get_planning_scene",
            callback_group=self._callback_group
        )
        self._get_planning_scene_future = None
        self._set_planning_scene_srv = self._nh.create_client(
            srv_type=ApplyPlanningScene,
            srv_name="apply_planning_scene",
            callback_group=self._callback_group
        )

    # --- Moveit Commander (Missing) Methods
    def _get_known_object_poses(self, with_type: bool = False) -> dict:
        """Gets known objects from the planning scene service

        :param with_type: Retrieve type if True, defaults to False
        :type with_type: bool, optional
        :return: Dictionary of string names and their poses
        :rtype: dict
        """
        result = dict()
        req = GetPlanningScene.Request()
        req.components.components = req.components.WORLD_OBJECT_GEOMETRY

        # --- Check if service is available and send path request  
        if self._get_planning_scene_future and self._get_planning_scene_future.done():
            resp = self._get_planning_scene_future.result()

            for object in resp.scene.world.collision_objects:
                result[object.id] = object.pose 

            self._get_planning_scene_future = None
            self._logger.info(
                f"[ArmCommander::_get_known_object_poses][Received: {result}]"
            )

            return result
        else:
            # Make a request
            self._get_planning_scene_future = self._get_planning_scene_srv.call_async(req)
            return dict()
    
    # --- Internal Methods
    def _move_default_goal(
            self, 
            frame_id: str = None, 
            group_name: str = None, 
            end_effector: str = None) -> MoveGroup.Goal:
        """Creates a default move group goal based on input params

        :param frame_id: Base frame ID, defaults to None
        :type frame_id: str, optional
        :param group_name: Move group name, defaults to None
        :type group_name: str, optional
        :param end_effector: End effector link name, defaults to None
        :type end_effector: str, optional
        :return: Move group goal object
        :rtype: MoveGroup.Goal
        """
        move_goal = MoveGroup.Goal()

        # --- Workspace Setup
        move_goal.request.workspace_parameters.header.frame_id = frame_id
        move_goal.request.workspace_parameters.min_corner.x = -1.0
        move_goal.request.workspace_parameters.min_corner.y = -1.0
        move_goal.request.workspace_parameters.min_corner.z = -1.0
        move_goal.request.workspace_parameters.max_corner.x = 1.0
        move_goal.request.workspace_parameters.max_corner.y = 1.0
        move_goal.request.workspace_parameters.max_corner.z = 1.0

        # --- Create empty constraints list for population
        move_goal.request.goal_constraints = [self._the_constraints]

        # --- Property setup
        # TODO: read from config
        move_goal.request.group_name = group_name
        move_goal.request.num_planning_attempts = 10
        move_goal.request.allowed_planning_time = 5.0 #sec
        move_goal.request.max_velocity_scaling_factor = 0.1
        move_goal.request.max_acceleration_scaling_factor = 0.1
        move_goal.request.cartesian_speed_end_effector_link = end_effector
        move_goal.request.max_cartesian_speed = 0.01 #m/s
        
        # --- Planning setup
        move_goal.planning_options.plan_only = False

        return move_goal
    
    def _cb_joint_state(self, msg: JointState) -> None:
        """Callback on joint state information

        :param msg: joint state message 
        :type msg: JointState
        """
        # TODO: handle check for joint names dependent on robot

        self._joint_state_mutex.acquire()
        self._joint_state = msg
        self._joint_state_mutex.release()        

    def _cb_timer(self) -> None:
        """Timer for waking up IDLE commander
        """
        self._pub_workspace_color()
        self._pub_transform_all_objects()

    def _pub_workspace_color(self) -> None:
        """TODO: publish colours of workspace (wall) objects
        """
        # self._logger.warn(f"[ArmCommander::_pub_workspace_color][Not Yet Implemented]")
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        for wall in self._wall_name_list:
            color = self._set_object_color(wall, 0.6, 0, 1, 0.2)
            planning_scene.object_colors.append(color)
        self._planning_scene_pub.publish(planning_scene)

    def _pub_transform_all_objects(self) -> None:
        """TODO: publish transforms of all objects in scene
        """
        # self._logger.warn(f"[ArmCommander::_pub_transform_all_objects][Not Yet Implemented]")
        object_pose_dict = self._get_known_object_poses()
        for name, pose in object_pose_dict.items():
            self._pub_transform_object(name=name, pose=pose)

    def _pub_transform_object(self, name, pose) -> None:
        """Publish transform of a specific named object

        :param name: _description_
        :type name: _type_
        :param pose: _description_
        :type pose: _type_
        """
        tf = Transform()
        if type(pose) == Pose:
            tf.translation.x = pose.position.x
            tf.translation.y = pose.position.y
            tf.translation.z = pose.position.z
            tf.rotation.x = pose.orientation.x
            tf.rotation.y = pose.orientation.y
            tf.rotation.z = pose.orientation.z
            tf.rotation.w = pose.orientation.w
        elif type(pose) == PoseStamped:
            tf.translation.x = pose.pose.position.x
            tf.translation.y = pose.pose.position.y
            tf.translation.z = pose.pose.position.z
            tf.rotation.x = pose.pose.orientation.x
            tf.rotation.y = pose.pose.orientation.y
            tf.rotation.z = pose.pose.orientation.z
            tf.rotation.w = pose.pose.orientation.w
        else:
            raise ValueError(f"Parameter pose should be a Pose or PoseStamped object")
        
        tf_stamped = TransformStamped()
        tf_stamped.child_frame_id = name
        tf_stamped.header.frame_id = self._world_reference_link
        tf_stamped.header.stamp = self._nh.get_clock().now().to_msg()
        tf_stamped.transform = tf
        self._tf_pub.sendTransform(transform=tf_stamped)

    def _cb_move_group_result(self) -> None:
        """TODO: alternative direct API version of this
        """
        self._logger.warn(f"[ArmCommander::_cb_move_group_result][Not Yet Implemented]")

    def _cb_trajectory_execute_result(self) -> None:
        """TODO: alternative direct API version of this
        """
        self._logger.warn(f"[ArmCommander::_cb_trajectory_execute_result][Not Yet Implemented]")

    def _cb_handle_result_status(self) -> None:
        """TODO: alternative direct API version of this

        :return: _description_
        :rtype: _type_
        """
        self._logger.warn(
            f"[ArmCommander::_cb_handle_result_status][Not Yet Implemented]"
        )

    def _cb_execute_move_action(self, outcome) -> None:
        """Callback from a move_group Execution Request

        :param outcome: _description_
        :type outcome: _type_
        """
        goal_handle = outcome.result()
        if not goal_handle.accepted:
            self._logger.warn(
                f"[ArmCommander::_cb_execute_move_action][<{self._move_action_client._action_name}> was rejected]"
            )
            return None
        
        # Track handle for abort call
        self._cb_execute_goal_handle = goal_handle

        # Get result from execution
        self._logger.info(
            f"[ArmCommander::_cb_execute_move_action][<{self._move_action_client._action_name}> getting result...]"
        )
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._cb_execute_move_action_result)

    def _cb_execute(self, outcome) -> None:
        """Callback to Trajectory Execution Manager execution call

        :param outcome: Status of execution
        :type outcome: ExecutionStatus
        """
        goal_handle = outcome.result()
        if not goal_handle.accepted:
            self._commander_state = CommanderStates.ABORTED
            self._logger.warn(
                f"[ArmCommander::_cb_execute][{self._follow_jt_action_client._action_name} was rejected]"
            )
            return None
        
        # Track handle for abort call
        self._cb_execute_goal_handle = goal_handle

        # Get the result from the execution
        self._logger.info(
            f"[ArmCommander::_cb_execute][{self._follow_jt_action_client._action_name} getting result...]"
        )
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._cb_execute_result)

    def _cb_execute_result(self, future):
        """Callback to result from Trajectory Execution

        :param future: Result Future
        :type future: _type_
        """
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._logger.error(
                f"[ArmCommander::_cb_execute_result][{self._follow_jt_action_client._action_name} was unsuccessful <{future.result().status}>]"
            )

            self._commander_state = CommanderStates.ERROR
        else:
            self._logger.info(
                f"[ArmCommander::_cb_execute_result][{self._follow_jt_action_client._action_name} SUCCEEDED]"
            )

            self._commander_state = CommanderStates.SUCCEEDED

    def _cb_execute_move_action_result(self, future):
        """Callback to result from Move Group Trajectory Execution

        :param future: Result Future
        :type future: _type_
        """
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._logger.error(
                f"[ArmCommander::_cb_execute_move_action_result][{self._move_action_client._action_name} was unsuccessful <{future.result().status}>]"
            )

            self._commander_state = CommanderStates.ERROR
        else:
            self._logger.info(
                f"[ArmCommander::_cb_execute_move_action_result][{self._move_action_client._action_name} SUCCEEDED]"
            )

            self._commander_state = CommanderStates.SUCCEEDED

    def _cb_execute_set_event(self, outcome) -> None:
        """Callback to Trajectory Execution call (on event wait)

        :param outcome: _description_
        :type outcome: _type_
        :return: None
        :rtype: None
        """
        self._logger.info(
            f"[ArmCommander::_cb_execute_set_event][Waiting on Completion...]"
        )
        self._cb_execute(outcome=outcome)
        self._future.set()

    def _cb_execute_cancel(self, future) -> None:
        """Callback on goal cancel request

        :param future: _description_
        :type future: _type_
        :raises ValueError: _description_
        :return: _description_
        :rtype: _type_
        """
        cancel_resp = future.result()
        if len(cancel_resp.goals_canceling) > 0:
            self._commander_state = CommanderStates.ABORTED
            self._logger.info(
                f"[ArmCommander::_cb_execute_cancel][Goal Cancelled Successfully]"
            )
        else:
            self._commander_state = CommanderStates.ERROR
            self._logger.error(
                f"[ArmCommander::_cb_execute_cancel][Failed]"
            )

    # --- Utility Methods
    def add_named_pose(self, name: str, joint_values: list) -> None:
        """Utility method to add a new named pose

        :param name: Key name of joints for future use
        :type name: str
        :param joint_values: Joint values of robot to save
        :type joint_values: list
        """
        if name in self._named_poses.keys():
            self._logger.warn(
                f"[ArmCommander::add_named_pose][<{name}> already saved]"
            )
        
        # Set new named key with joint values
        self._logger.info(
            f"[ArmCommander::add_named_pose][Added <{name}> with joints: {joint_values}]"
        )
        self._named_poses[name] = joint_values

    def forget_named_pose(self, name: str) -> None:
        """Utility method to remove/forget a named pose

        :param name: Key name of joints saved
        :type name: str
        """
        if name not in self._named_poses.keys():
            self._logger.warn(
                f"[ArmCommander::forget_named_pose][<{name}> not in saved list to forget]"
            )
        
        self._logger.info(
            f"[ArmCommander::forget_named_pose][Forgetting <{name}>]"
        )
        del self._named_poses[name]

    def spin(self) -> None:
        """Spins up the ROS2 Node
        """
        rclpy.spin(self._nh)

    def shutdown(self) -> None:
        """Shutdown command for ROS2

        :return: None
        :rtype: None
        """
        rclpy.shutdown()

    def reset_state(self) -> None:
        """Reset of states and variables on command

        :return: None
        :rtype: None
        """
        # -- Debugging check on current state (prior to reset)
        self._logger.info(
            f"[ArmCommander::reset_state][Resetting from <{self._commander_state}>]"
        )
        # -- set state
        self._commander_state = CommanderStates.READY
        self._cached_result = None 

        return None

    def wait_for_busy_end(self) -> CommanderStates:
        """Wait method while robot is busy

        :return: The final state of the commander
        :rtype: CommanderStates
        """
        while True:
            if self._commander_state not in [CommanderStates.BUSY]:
                return self._commander_state
            self._wait_for_busy_end_rate.sleep() 

    def abort_move(self, wait: bool = True) -> bool:
        """Aborts motion on command

        :param wait: Waits for completion if True, defaults to True
        :type wait: bool, optional
        :return: True on success, else False
        :rtype: bool
        """
        # Request stop if in motion
        if self._cb_execute_goal_handle:
            future = self._cb_execute_goal_handle.cancel_goal_async()
            future.add_done_callback(self._cb_execute_cancel)
            self._logger.info(f"[ArmCommander::abort_move][Stop Requested]")
        else:
            self._logger.warn(f"[ArmCommander::abort_move][Nothing to Stop]")
        
        if wait:
            self.wait_for_busy_end()

        self.reset_state()
        
        return True
    
    # --- Robot Movement Methods
    def _execute(self, trajectory: JointTrajectory = None, wait: bool = False) -> None:
        """A common plan and execute method for the robot

        :param wait: Set True to wait for execution, defaults to False
        :type wait: bool, optional
        :return: None (outcome in callback -> see ArmCommander::cb_plan_execute_status())
        :rtype: None
        """
        # Check if controller is already running before requesting new plan

        # Create a trajectory goal
        jt_goal = FollowJointTrajectory.Goal()
        jt_goal.trajectory = trajectory

        # --- Execute on Successful Plan
        if jt_goal:
            # Check if service is available
            if not self._follow_jt_action_client.wait_for_server(timeout_sec=1):
                self._logger.warn(
                    f"[ArmCommander::_execute][{self._follow_jt_action_client._action_name} not available]"
                )
                return None
            
            self._logger.info(
                f"[ArmCommander::_execute][Goal sent for execution]"
            )
            resp = self._follow_jt_action_client.send_goal_async(goal=jt_goal, feedback_callback=None)

            # Add a completion callback depending on wait
            if wait:
                self._future.clear()
                resp.add_done_callback(self._cb_execute_set_event)
                self._future.wait(timeout=1)

                # --- Wait for completion
                self.wait_for_busy_end()

                self._logger.info(
                    f"[ArmCommander::_execute][COMPLETED <WAIT>]"
                )
            else:
                resp.add_done_callback(self._cb_execute)

                self._logger.info(
                    f"[ArmCommander::_execute][COMPLETED <NO WAIT>]"
                )
        else:
            self._commander_state = CommanderStates.ABORTED
            self._logger.warn(
                f"[ArmCommander::_execute][Trajectory/plan is invalid]"
            )
    
    def _plan_and_execute_move_group(self, plan_only: bool = False) -> None:
        """Planning call to move_group

        :param plan_only: Flag to do only planning and no execution
        :type plan_only: bool, optional
        :return: None
        :rtype: None
        """
        # --- Set current planning options
        self._move_action_goal.planning_options.plan_only = plan_only
        self._move_action_goal.request.workspace_parameters.header.stamp = self._nh.get_clock().now().to_msg()

        # --- Check for service ready
        if not self._move_action_client.wait_for_server(timeout_sec=1.0):
            self._logger.warn(
                f"[ArmCommander::_execute_move_group][<{self._move_action_client._action_name}> Not Ready]"
            )
            return None
    
        # --- Send goal for plan/execution/both
        move_action_resp = self._move_action_client.send_goal_async(
            goal=self._move_action_goal,
            feedback_callback=None,
        )

        move_action_resp.add_done_callback(self._cb_execute_move_action)

    def _plan(self, 
            accept_fraction: float = 0.9999, 
            cartesian: bool = False, 
            target_pose: PoseStamped = None) -> JointTrajectory:
        """Class method to plan for a trajectory execution

        NOTE: may not be needed at this level of abstraction
        :param accept_fraction: the acceptable minimum fraction of the planned path, defaults to 0.9999
        :type accept_fraction: float, optional
        :param cartesian: Plan a cartesian path if True
        :type cartesian: bool, optional
        :param target_pose: Target pose that has been requested
        :type target_pose: PoseStamped
        :return: Valid trajectory if plan was successfull
        :rtype: JointTrajectory
        """
        plan = None
        if not target_pose:
            self._logger.warn(
                f"[ArmCommander::_plan][Target pose is not defined]"
            )
            return plan
        
        # --- Construct a plan request depending on cartesian or kinematic request
        if cartesian:
            # --- Setup cartesian path request
            return self._plan_cartesian_path(target_pose=target_pose, accept_fraction=accept_fraction)
        else:
            # --- Setup a normal motion path request
            return self._plan_kinematic_path()
            
    def _plan_cartesian_path(self, target_pose: PoseStamped, accept_fraction: float = 0.9999) -> JointTrajectory:
        """Internal method to plan a cartesian path (for direct controller usage)

        :param target_pose: Target pose to aim for
        :type target_pose: PoseStamped
        :param accept_fraction: the acceptable minimum fraction of the planned path, defaults to 0.9999
        :type accept_fraction: float, optional
        :return: Joint trajectory of a successful plan (None otherwise)
        :rtype: JointTrajectory
        """
        self._logger.info(
            f"[ArmCommander::_plan_cartesian_path][CARTESIAN path requested!]"
        )
        cart_req = GetCartesianPath.Request()
        cart_req.start_state = self._move_action_goal.request.start_state
        cart_req.group_name = self._move_action_goal.request.group_name
        cart_req.link_name = self._end_effector_link
        cart_req.max_step = self.CARTE_PLANNING_STEP_SIZE
        cart_req.header.frame_id = self._world_reference_link
        cart_req.header.stamp = self._nh.get_clock().now().to_msg()
        cart_req.path_constraints = self._move_action_goal.request.path_constraints
        cart_req.waypoints = [target_pose.pose]
        cart_req.avoid_collisions = True

        # --- Update each constraint stamp
        for con in cart_req.path_constraints.position_constraints:
            con.header.stamp = self._nh.get_clock().now().to_msg()
        for con in cart_req.path_constraints.orientation_constraints:
            con.header.stamp = self._nh.get_clock().now().to_msg()

        # --- Check if service is available and send path request            
        if not self._plan_cartesian_path_srv.wait_for_service(timeout_sec=1):
            self._logger.warn(
                f"[ArmCommander::_plan_cartesian_path][Cartesian plan service unavailable"
            )
            return None
        
        resp = self._plan_cartesian_path_srv.call(cart_req)

        if resp.error_code.val == MoveItErrorCodes.SUCCESS:
            plan = resp.solution.joint_trajectory
            self._logger.info(
                f"[ArmCommander::_plan_cartesian_path][CARTESIAN path determined!]"
            )

            # --- Validate acceptable fraction
            if resp.fraction < accept_fraction:
                self._logger.warn(
                    f"[ArmCommander::_plan_cartesian_path][CARTESIAN Fraction Invalid <{resp.fraction}> less than <{accept_fraction}>]"
                )
                self._commander_state = CommanderStates.ABORTED
                return None

            return plan
        else:
            self._logger.warn(
                f"[ArmCommander::_plan_cartesian_path][Could not plan cartesian path <{resp.error_code.val}>]"
            )
            return None
    
    def _plan_kinematic_path(self) -> JointTrajectory:
        """Internal method to plan a kinematic path (for direct controller usage)

        :return: Joint trajectory of a successful plan (None otherwise)
        :rtype: JointTrajectory
        """
        self._logger.info(
            f"[ArmCommander::_plan_kinematic_path][KINEMATIC path requested!]"
        )
        kine_req = GetMotionPlan.Request()
        kine_req.motion_plan_request = self._move_action_goal.request
        kine_req.motion_plan_request.workspace_parameters.header.stamp = self._nh.get_clock().now().to_msg()

        # --- Update each constraint stamp
        for con in kine_req.motion_plan_request.goal_constraints:
            for pos_con in con.position_constraints:
                pos_con.header.stamp = self._nh.get_clock().now().to_msg()
            for rot_con in con.orientation_constraints:
                rot_con.header.stamp = self._nh.get_clock().now().to_msg()

        # --- Check if service is available and send motion request
        if not self._plan_kinematic_path_srv.wait_for_service(timeout_sec=1):
            self._logger.warn(
                f"[ArmCommander::_plan_kinematic_path][Plan service unavailable"
            )
            return None

        resp = self._plan_kinematic_path_srv.call(kine_req).motion_plan_response

        if resp.error_code.val == MoveItErrorCodes.SUCCESS:
            plan = resp.trajectory.joint_trajectory
            self._logger.info(
                f"[ArmCommander::_plan_kinematic_path][Path determined!]"
            )
            return plan
        else:
            self._logger.warn(
                f"[ArmCommander::_plan_kinematic_path][Could not plan path <{resp.error_code.val}>]"
            )
            self._commander_state = CommanderStates.ABORTED
            return None

    def _set_constraints(
            self,
            type: ConstraintOptions = ConstraintOptions.POSITION, 
            target_pose: PoseStamped = None,
            joint_poses: list[float] = None,
            tolerance: float = 0.001,
            weight: float = 0.1) -> None:
        """Sets position constraints based on inputs

        :param type: Constraint type to set [Position, Orientation]
        :type type: ConstraintOptions
        :param target_pose: Target pose to set, defaults to None
        :type target_pose: PoseStamped
        :param tolerance: Goal tolerance (m), defaults to 0.001
        :type tolerance: float, optional
        :param weight: Weight of goal, defaults to 0.1
        :type weight: float, optional
        """
        if type == ConstraintOptions.POSITION:
            con = PositionConstraint()
        elif type == ConstraintOptions.ORIENTATION:
            con = OrientationConstraint()
        elif type == ConstraintOptions.JOINT:
            # Utilised more below
            pass
        else:
            self._logger.warn(
                    f"[ArmCommander::_set_constraints][Unknown Constraint type: {type}]"
                )
            return None
        
        # --- Common settings
        con.header.frame_id = self._world_reference_link
        con.link_name = self._end_effector_link

        if type == ConstraintOptions.POSITION:
            con.constraint_region.primitive_poses.append(Pose())
            con.constraint_region.primitive_poses[0].position = target_pose.pose.position
            # Define goal region as sphere of radius tolerance
            con.constraint_region.primitives.append(SolidPrimitive())
            con.constraint_region.primitives[0].type = 2 #Sphere
            con.constraint_region.primitives[0].dimensions = [tolerance] #m
            con.weight = weight
            self._move_action_goal.request.goal_constraints[-1].position_constraints.append(con)
        elif type == ConstraintOptions.ORIENTATION:
            con.orientation = target_pose.pose.orientation
            # Defin goal tolerances
            con.absolute_x_axis_tolerance = 0.001
            con.absolute_y_axis_tolerance = 0.001
            con.absolute_z_axis_tolerance = 0.001
            con.weight = weight
            self._move_action_goal.request.goal_constraints[-1].orientation_constraints.append(con)
        elif type == ConstraintOptions.JOINT:
            # --- Iterate through each joint and add constraint
            for i in range(len(joint_poses)):
                con = JointConstraint()
                con.position = joint_poses[i]
                con.tolerance_above = tolerance
                con.tolerance_below = tolerance
                con.weight = weight
                self._move_action_goal.request.goal_constraints[-1].joint_constraints.append(con)
            
    def move_to_named_pose(self, named_pose: str = "", wait: bool = True) -> None:
        """Move to a Named Pose on Command

        :param named_pose: Name of pose to move to, defaults to ""
        :type named_pose: str, optional
        :param wait: Waits for completion if True, defaults to True
        :type wait: bool, optional
        """
        self._action_lock.acquire()
        try:
            if self._commander_state != CommanderStates.READY:
                self._logger.error(
                    f"[ArmCommander::move_to_named_pose][Commander NOT READY]"
                )
                return None

            # Get name from saved list (if available)
            if named_pose not in self._named_poses.keys():
                self._logger.error(
                    f"[ArmCommander::move_to_named_pose][Cannot retreive joints for <{named_pose}>]"
                )
                return None
            
            # --- Set joint constraints
            self._set_constraints(type=ConstraintOptions.JOINT, joint_poses=self._named_poses[named_pose])

            # --- Create Default Plan and Execute  
            # NOTE: Execute directly through move_group node
            self._logger.info(
                f"[ArmCommander::move_to_named_pose][Executing...]"
            )
            self._commander_state = CommanderStates.BUSY
            self._plan_and_execute_move_group(plan_only=False) 

            # --- Clearing constraints for next cycle
            # self._the_constraints = Constraints()
            self._move_action_goal.request.goal_constraints = [Constraints()]            
        finally:
            self._action_lock.release()
    
    def move_displacement(self,
                        dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                        accept_fraction: float = 0.9999, wait: bool = True) -> Pose:
        """Command the robot arm to move the end effector by a displacement (dx, dy, dz)
        in a Cartesian motion manner

        :param dx: Displacement in x, defaults to 0.0
        :type dx: float, optional
        :param dy: Displacement in y, defaults to 0.0
        :type dy: float, optional
        :param dz: Displacement in z, defaults to 0.0
        :type dz: float, optional
        :param accept_fraction: Acceptabl minimum fraction of planned path, defaults to 0.9999
        :type accept_fraction: float, optional
        :param wait: Wait call to block until completion (if True), defaults to True
        :type wait: bool, optional
        :return: The target pose
        :rtype: Pose
        """
        self._action_lock.acquire()
        try:
            if self._commander_state != CommanderStates.READY:
                self._logger.error(
                    f"[ArmCommander::move_displacement][Commander NOT READY]"
                )
                return None
            
            # --- Get the current pose of the end-effector link
            ee_pose: PoseStamped = self.pose_in_frame(
                link_name=self._end_effector_link, 
                reference_frame=self._world_reference_link
            )

            # --- Copy and update current ee pose with displacement
            target_pose = copy.deepcopy(ee_pose)
            target_pose.pose.position.x += dx
            target_pose.pose.position.y += dy
            target_pose.pose.position.z += dz

            self._logger.info(
                f"[ArmCommander::move_displacement][Requested Plan and Execution on {target_pose.pose}]"
            )

            # --- Set position constraints
            self._set_constraints(type=ConstraintOptions.POSITION, target_pose=target_pose)

            # --- Set orientation constratins
            self._set_constraints(type=ConstraintOptions.ORIENTATION, target_pose=target_pose)

            # --- Set starting joint state (curent from callback)
            # NOTE: this may not be available -> resulting in a failed plan
            if self._joint_state is not None:
                self._move_action_goal.request.start_state.joint_state = self._joint_state

            # --- Request plan 
            # NOTE: default cartesian request
            plan = self._plan_cartesian_path(target_pose=target_pose, accept_fraction=accept_fraction)
            if not plan:
                self._logger.warn(
                    f"[ArmCommander::move_displacement][Could not create plan]"
                )
                self._commander_state = CommanderStates.ABORTED
                return None                

            # --- Create Default Plan and Execute with/without Wait  
            # NOTE: execute directly with controller
            self._logger.info(
                f"[ArmCommander::move_displacement][Executing...]"
            )
            self._commander_state = CommanderStates.BUSY
            self._execute(trajectory=plan, wait=wait) 

            # --- Clearing constraints for next cycle
            # self._the_constraints = Constraints()
            self._move_action_goal.request.goal_constraints = [Constraints()]

        except Exception as e:
            self._logger.error(
                f"[ArmCommander::move_displacement][System error <{e} {traceback.format_exc()}>]"
            )
            self._commander_state = CommanderStates.ABORTED
            self._commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'
        finally:
            self._action_lock.release()

    def move_to_position(self, x: float = None, y: float = None, z: float = None, 
                         accept_fraction: float = 0.9999, cartesian: bool = False, 
                         reference_frame: str = None, wait: bool = True) -> PoseStamped:
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
        :rtype: PoseStamped
        """
        if self._commander_state != CommanderStates.READY:
            self._logger.warn(
                f"[ArmCommander::move_to_position][Commander NOT READY]"
            )
            return None
        
        self._action_lock.acquire()
        try:
            # --- Configure target pose from inputs
            reference_frame = reference_frame if reference_frame is not None else self._world_reference_link
            target_pose: PoseStamped = self.pose_in_frame(self._end_effector_link, reference_frame)
            target_pose.pose.position.x = x if x is not None else target_pose.pose.position.x
            target_pose.pose.position.y = y if y is not None else target_pose.pose.position.y
            target_pose.pose.position.z = z if z is not None else target_pose.pose.position.z
            self._logger.info(
                f"[ArmCommander::move_to_position][Requested Plan and Execution on {target_pose.pose}]"
            )

            # --- Set position constraints
            self._set_constraints(type=ConstraintOptions.POSITION, target_pose=target_pose)

            # --- Set orientation constratins
            self._set_constraints(type=ConstraintOptions.ORIENTATION, target_pose=target_pose)

            # Set starting joint state (curent from callback)
            # NOTE: this may not be available -> resulting in a failed plan
            if self._joint_state is not None:
                self._move_action_goal.request.start_state.joint_state = self._joint_state

            # Request plan (cartesian or otherwise)
            # if cartesian:
            plan = self._plan(cartesian=cartesian, accept_fraction=accept_fraction, target_pose=target_pose)
            if not plan:
                self._logger.warn(
                    f"[ArmCommander::move_to_position][Could not create plan]"
                )
                self._commander_state = CommanderStates.ABORTED
                return None                

            # --- Create Default Plan and Execute with/without Wait  
            self._logger.info(
                f"[ArmCommander::move_to_position][Executing...]"
            )
            self._commander_state = CommanderStates.BUSY
            self._execute(trajectory=plan, wait=wait) 
            # else:
            #     # self._logger.info(
            #     #     f"[ArmCommander::move_to_named_pose][Executing...]"
            #     # )
            #     # self._commander_state = CommanderStates.BUSY
            #     # self._plan_and_execute_move_group(plan_only=False) 

            # --- Clearing constraints for next cycle
            # self._the_constraints = Constraints()
            self._move_action_goal.request.goal_constraints = [Constraints()]

        except Exception as e:
            self._logger.error(
                f"[ArmCommander::move_to_position][System Error <{e} {traceback.format_exc()}>]"
            )
        self._action_lock.release()

        return target_pose 

    # --- Robot Query Methods
    def current_joint_positions(self, print=False) -> dict:
        """ Get the positions of all the joints

        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: Pairs of (joint_name, position) in a dictionary
        :rtype: dict
        """
        self._logger.warn(f"[ArmCommander::current_joint_positions][Not Yet Implemented]")
        # TODO: update this method for moveit_py
        joint_values_dict = dict() #self.robot.get_current_variable_values()
        if print: self._logger.info(joint_values_dict)
        return joint_values_dict
    
    def current_joint_positons_as_list(self, print=False) -> list:
        """ Get the positions of all the joints

        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: Lists of joint positions
        :rtype: list
        """
        self._logger.warn(f"[ArmCommander::current_joint_positions_as_list][Not Yet Implemented]")
        # TODO: update this method for moveit_py
        joint_values = list() #self.move_group.get_current_joint_values()
        if print: self._logger.info(joint_values)
        return joint_values   

    def pose_in_frame_as_xyzq(self, 
                        link_name: str = None, 
                        reference_frame: str = None, 
                        ros_time: rclpy.time.Time = None, 
                        print: bool = False) -> list:
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
        link_name = self._end_effector_link if link_name is None else link_name
        ros_time = rclpy.time.Time() if ros_time is None else ros_time
        reference_frame = self._world_reference_link if reference_frame is None else reference_frame    
        try:
            trans = self._tf_buffer.lookup_transform(
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
                self._logger.info(f'[ArmCommander::pose_in_frame_as_xyzq][pose of {link_name} from {reference_frame}: {trans_list}]')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            raise ValueError(f'Invalid parameter or TF error: {e}')
        except tf2_ros.ExtrapolationException as e:
            self._logger.error(f"[ArmCommander::pose_in_frame_as_xyzq][Extrapolation Error: {e}]")
            raise
        
        return trans_list
    
    def pose_in_frame_as_xyzrpy(self, 
                        link_name: str = None, 
                        reference_frame: str = None, 
                        ros_time: rclpy.time.Time = None, 
                        print: bool = False) -> list:
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
        link_name = self._end_effector_link if link_name is None else link_name
        ros_time = rclpy.time.Time() if ros_time is None else ros_time
        reference_frame = self._world_reference_link if reference_frame is None else reference_frame 

        # Request 7 float list of pose
        pose = self.pose_in_frame_as_xyzq(link_name, reference_frame, ros_time)

        # Extract and extend list with rpy information
        xyzrpy = pose[:3]
        xyzrpy.extend(tf_transformations.euler_from_quaternion(pose[3:]))
        if print: 
            self._logger.info(f'[ArmCommander::pose_in_frame_as_xyzrpy][pose of {link_name} from {reference_frame}: {xyzrpy}]')
        return xyzrpy
    
    def pose_in_frame(self, 
                link_name: str = None, 
                reference_frame: str = None, 
                ros_time: rclpy.time.Time = None) -> PoseStamped:
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
        reference_frame = self._world_reference_link if reference_frame is None else reference_frame        
        link_name = self._end_effector_link if link_name is None else link_name
        ros_time = rclpy.time.Time() if ros_time is None else ros_time

        # Request 7 float list of pose
        xyzq = self.pose_in_frame_as_xyzq(link_name, reference_frame, ros_time)

        # Construct PoseStamped Message
        # NOTE: time stamp populated from node handle
        pose_stamped_out = PoseStamped()
        pose_stamped_out.header.frame_id = reference_frame
        pose_stamped_out.header.stamp = self._nh.get_clock().now().to_msg()

        if len(xyzq) != 7:
            self._logger.error(f"[ArmCommander::pose_in_frame][size of xyzq is invalid: {len(xyzq)}]")
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
        pose_stamped.header.frame_id = self._world_reference_link
        pose_stamped.header.stamp = self._nh.get_clock().now().to_msg()
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
            robot_link_name = self._end_effector_link

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
        if print: self._logger.info(f"[ArmCommander::rpy_of_robot_link][{rpy}]")
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
            robot_link_name = self._end_effector_link
            
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
        if print: self._logger.info(f"[ArmCommander::xyzrpy_of_robot_link][{xyzrpy}]")
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
            transformed_pose = self._tf_buffer.transform(
                object_stamped=the_pose,
                target_frame=to_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.ExtrapolationException as e:
            self._logger.error(f"[ArmCommander::transform_pose][Extrapolation Error: {e}]")
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
        self._logger.warn(f"[ArmCommander::add_named_pose][Not Yet Implemented]")
        # self.move_group.remember_joint_values(name, values=joint_values)
    
    # remove the named pose from the commander
    def forget_named_pose(self, name: str):
        """Remove the named pose from the commander

        :param name: The name of the pose to be defined
        :type name: str
        """
        self._logger.warn(f"[ArmCommander::add_named_pose][Not Yet Implemented]")
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
                link_name = self._end_effector_link

            # Get the current pose from the robot state
            # robot_state = RobotState(self.robot.get_robot_model())
            # robot_state.update()

            pose = Pose() #robot_state.get_pose(link_name)
            if print: self._logger.info(f"[ArmCommander::_get_pose][Link {link_name} has current_pose: {pose}]")
        else:
            self._logger.error(f"[ArmCommander::_get_pose][Move group or end-effector link invalid]")
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
        f'> group name: {self._move_group_name}',
        f'> planning time: {nyi}',
        f'> pose reference frame: {self._world_reference_link}',   
        f'> end-effector:\t\nlinks: {self._end_effector_link}',
        f'> pose: {self.pose_of_robot_link().pose}',
        f'> roll, pitch, yaw: {nyi}',
        f'> goal tolerance (joint, position, orientation): {nyi}'
        ]
        output = '\n'.join(string_list)
        if print:
            self._logger.info(output)
        return output
    
    def get_latest_moveit_feedback(self) -> None:
        """Get the latest feedback resulting from the last command

        :return: The latest action feedback of the last call to the underlying move commander
        :rtype: None
        """
        self._logger.warn(f"[ArmCommander::get_latest_moveit_feedback][Not Yet Implemented]")
        return self._cached_result
    
    def get_commander_state(self, print=False) -> CommanderStates:
        """Get the current state of this general commander

        :param print: Sends the state to the output as well, defaults to False
        :type print: bool, optional
        :return: The current state of this general commander
        :rtype: CommanderStates
        """
        if print:
            self._logger.info(self._commander_state.name)
        return self._commander_state
    
    def get_error_code(self, print=False) -> None:
        """Get the error code of resulting from the last command

        :param print: Sends the error code to the output as well, defaults to False
        :type print: bool, optional
        :return: The error code of the last command
        :rtype: None
        """
        self._logger.warn(f"[ArmCommander::get_error_code][Not Yet Implemented]")
        if self._cached_result is None:
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
        self._logger.warn(f"[ArmCommander::get_goal_tolerance][Not Yet Implemented]")
        return None #self.move_group.get_goal_tolerance()

    # --- General Set Methods
    def set_max_cartesian_speed(self, max_speed: float = None):
        """Set the maximum speed of the end-effector motion

        :param max_speed: The maximum speed in meters per seconds, defaults to the system default speed
        :type max_speed: float, optional
        """
        self._logger.warn(f"[ArmCommander::set_max_cartesian_speed][Not Yet Implemented]")

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
        self._logger.warn(f"[ArmCommander::set_planning_time][Not Yet Implemented]")
        # self.move_group.set_planning_time(planning_time)
    
    # set the goal tolerance (a list of joint, position and orientation goal tolerances)
    def set_goal_tolerance(self, tol_list: list): 
        """Set the goal tolerance 

        :param tol_list: set the goal tolerance as a list of joint, position and orientation goal tolerances
        :type tol_list: list
        """
        self._logger.warn(f"[ArmCommander::set_goal_tolerance][Not Yet Implemented]")
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
                self._end_effector_link = link_list[-1]
                self._logger.info(f"[ArmCommander::set_ee_link][Successfully set ee link: {self._end_effector_link}]")
            else:
                # Update based on provided link name as it is in link list
                self._end_effector_link = link_name
        else:
            self._logger.error(f"[ArmCommander::set_ee_link][move_group not valid, cannot set ee link]")
            return False

    # --- Error Checking Methods
    def is_ee_link_valid(self) -> bool:
        """Confirms if an end-effector link has been set

        :return: True if set, or False
        :rtype: bool
        """
        if self._end_effector_link == None:
            return False
        else:
            return True
        
    def is_move_group_valid(self) -> bool:
        """Confirms if a move_group has been initialised
        - Returns True if set, or False

        :return: True or False depending on if move_group has been set
        :rtype: bool
        """
        if self._move_group == None:
            return False
        else:
            return True