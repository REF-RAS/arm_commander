#!/usr/bin/env python3
# Copyright 2024 - Andrew Kwok Fai LUI and Dasun GUNASINGHE
# Robotics and Autonomous Systems Group
# Research Engineering Facility, Queensland University of Technology (QUT)
#
# License: https://github.com/REF-RAS/arm_commander/blob/main/LICENSE

__author__ = 'Andrew Lui'
__author__ = 'Dasun Gunasinghe'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.1.0'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

import sys, copy, threading, time, signal, math, traceback, timeit, numbers, logging
import rospy, tf
from tf2_msgs.msg import TFMessage
import moveit_commander
import moveit_commander.conversions as conversions
from moveit_msgs.msg import MoveGroupActionFeedback, MoveGroupActionResult, MoveItErrorCodes, ExecuteTrajectoryActionResult
from moveit_msgs.msg import PlanningScene, ObjectColor 
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from controller_manager_msgs.srv import SwitchController

from arm_commander.moveit_tools import MOVEIT_ERROR_CODE_MAP, GOAL_STATUS_MAP
from arm_commander.states import GeneralCommanderStates, ControllerState

    
class GeneralCommander():
    """
    GeneralCommander: an interface for robot commander, with this class specifically interfacing
    to Moveit

    .. codeauthor:: Andrew Lui
    .. sectionauthor:: Andrew Lui
    """
    def __init__(self, moveit_group_name:str, world_link:str=None) -> None:
        """ Constructor of GeneralCommander. 
        Using the :class: 'commander_moveit.GeneralCommanderFactory' is recommended

        :param moveit_group_name: [description], the name of the manipulator defined in the moveit setup 
        :type moveit_group_name: str
        :param world_link: [description], the link name used as the frame of reference in moveit 
        :type world_link: str, optional
        """
        # check if a ROS node has been initialized 
        node_uri = rospy.get_node_uri()
        if node_uri is None:
            rospy.init_node('arm_commander_node', anonymous=False)
            rospy.logwarn(f'This application is not yet initialized as a ROS node.')
            rospy.logwarn(f'For the proper operation, the arm commander has called rospy.init_node() on behalf of the application.')
            rospy.logwarn(f'-> You may resolve this programming issue by adding the call at the start of the application.')
            rospy.logwarn(f'    rospy.init_node(\'my_node\', anonymous=False)')
        # initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        # create lock for synchronization
        self.action_lock = threading.Lock()
        # define constants (should import from config)
        self.TF_PUB_RATE = rospy.get_param('tf_rate', 10)  # default to 10 Hz
        self.CARTE_PLANNING_STEP_SIZE = rospy.get_param('step_size', 0.01)  # meters
        self.GROUP_NAME = moveit_group_name
        # initialize move_it variables 
        self.robot = moveit_commander.RobotCommander()
        self.scene:moveit_commander.PlanningSceneInterface = moveit_commander.PlanningSceneInterface()
        try: 
            self.move_group:moveit_commander.MoveGroupCommander = moveit_commander.MoveGroupCommander(self.GROUP_NAME)
        except RuntimeError as ex:
            rospy.logerr(f'Invalid move_group error in moveit MoveGroupCommander creation -> check if you have launched the right robot model or use the wrong group name')
            raise
        # initialize the world_link as the default planning frame
        self.END_EFFECTOR_LINK = self.move_group.get_end_effector_link()
        if world_link is not None:
            self.WORLD_REFERENCE_LINK = world_link
            rospy.loginfo(f'The default reference frame changed from {self.move_group.get_planning_frame()} to {world_link}')
            self.move_group.set_pose_reference_frame(world_link)
        else:
            self.WORLD_REFERENCE_LINK = self.move_group.get_planning_frame()
            rospy.loginfo(f'The default reference frame is {self.WORLD_REFERENCE_LINK}')            
        # subscribe to /move_group/feedback
        self.moveit_result_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, 
                                                    self._cb_move_group_result, queue_size=10)
        # subscribe to /move_group/feedback
        self.trajectory_execute_sub = rospy.Subscriber('/execute_trajectory/result', ExecuteTrajectoryActionResult, 
                                                    self._cb_trajectory_execute_result, queue_size=10)  
        # This is the publisher to the servo topic
        self.servo_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        # internal states             
        self.commander_state:GeneralCommanderStates = GeneralCommanderStates.INIT
        self.cached_result:MoveGroupActionResult = None
        # define the walls as the workspace limits
        self.wall_name_list = []

        # publisher for the planning scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        # the constraints
        self.the_constraints = Constraints()
        self.the_constraints.name = 'commander'
        # set timeout
        self.timer = rospy.Timer(rospy.Duration(1/self.TF_PUB_RATE), self._cb_timer)
    
    # -----------------------------------------------------
    # -- Internal functions for publishing states and poses
    
    # callback for waking up an IDLE robot agent
    def _cb_timer(self, event):
        self._pub_transform_all_objects()

    # publish colors of the wall objects 
    def _pub_workspace_color(self):
        p = PlanningScene()
        p.is_diff = True
        for wall in self.wall_name_list:
            color = self._set_object_color(wall, 0.6, 0, 1, 0.2)
            p.object_colors.append(color)
        self.scene_pub.publish(p)
    
    # publish the transform of all added objects
    def _pub_transform_all_objects(self):
        object_list = self.scene.get_known_object_names()
        object_poses = self.scene.get_object_poses(object_list)
        for object_name in object_list:
            pose_of_object = object_poses[object_name]
            self._pub_transform_object(object_name, pose_of_object)

    # publish the transform of a specific named object
    def _pub_transform_object(self, name, pose):
        if type(pose) == Pose:
            pose = conversions.pose_to_list(pose)
        elif type(pose) == PoseStamped:
            pose = conversions.pose_to_list(pose.pose)
        xyz = pose[:3]
        if type(pose) != list:
            rospy.logerr(f'{__class__.__name__}: parameter (pose) is not list of length 6 or 7 or a Pose object -> fix the parameter at behaviour construction')
            raise TypeError(f'A parameter is invalid')  
        if len(pose) == 6:
            q = tf.transformations.quaternion_from_euler([pose[3:]])
        elif len(pose) == 7:
            q = pose[3:]
        self.tf_pub.sendTransform(xyz, q, rospy.Time.now(), name, self.WORLD_REFERENCE_LINK)

    def _cb_move_group_result(self, msg:MoveGroupActionResult):
        rospy.loginfo(f'The commander (move_group result callback): {GOAL_STATUS_MAP[msg.status.status]} message: {MOVEIT_ERROR_CODE_MAP[msg.result.error_code.val]}')
        self._cb_handle_result_status(msg)

    def _cb_trajectory_execute_result(self, msg: ExecuteTrajectoryActionResult):
        rospy.loginfo(f'The commander (trajectory result callback): {GOAL_STATUS_MAP[msg.status.status]} message: {MOVEIT_ERROR_CODE_MAP[msg.result.error_code.val]}') 
        self._cb_handle_result_status(msg)
        
    def _cb_handle_result_status(self, msg):
        self.cached_result = msg
        if self.commander_state == GeneralCommanderStates.BUSY:
            if msg.status.status in [GoalStatus.SUCCEEDED]:
                self.commander_state = GeneralCommanderStates.SUCCEEDED
                self.commander_state.message = 'NO ERROR'
                return
            elif msg.status.status in [GoalStatus.ABORTED]:
                 self.commander_state = GeneralCommanderStates.ABORTED 
                 rospy.logerr(f'The commander: received ABORTED result')
            elif msg.status.status in [GoalStatus.PREEMPTED]:
                # just ignore the preempted goal - should not happen (TODO: assumption is wrong)
                self.commander_state = GeneralCommanderStates.ABORTED 
                rospy.logerr(f'The commander: the previous goal has been preempted')
            else:
                self.commander_state = GeneralCommanderStates.ERROR       
            self.commander_state.message = MOVEIT_ERROR_CODE_MAP[msg.result.error_code.val]
        else:
            rospy.logerr(f'The arm commander (cb_handle_result_status) received goal result {msg} in a wrong state {self.commander_state.name}')

    # -- Function for querying the generalCommander for various information

    # returns True when the General Commander is ready for servicing move commands
    def wait_for_ready_to_move(self, timeout=10.0) -> bool:
        """ returns True when the General Commander is ready for servicing move commands or otherwise if 
            the Commander is not ready after timeout
            
        :param timeout: _description_, defaults to 10.0
        :type timeout: float, optional
        :return: _description_
        :rtype: bool
        """
        try:
            rospy.wait_for_message('/tf', TFMessage, timeout)
            self.commander_state = GeneralCommanderStates.READY
            return True
        except:
            return False

    # returns the essential information of the robot move group (the arm) as a string
    # with an option to print to the screen
    def info(self, print=False) -> str:
        """Essential information for display

        :param print: Sends the information to the output, defaults to False
        :type print: bool, optional
        :return: Formatted information of the commander
        :rtype: str
        """
        string_list = [
        f'group name: {self.GROUP_NAME}',
        f'planning time: {self.move_group.get_planning_time()}',
        f'pose reference frame: {self.move_group.get_pose_reference_frame()}',   
        f'end-effector:\nlinks: {self.END_EFFECTOR_LINK}',
        f'pose: {self.move_group.get_current_pose(self.END_EFFECTOR_LINK).pose}',
        f'roll, pitch, yaw: {self.move_group.get_current_rpy(self.END_EFFECTOR_LINK)}',
        f'goal tolerance (joint, position, orientation): {self.move_group.get_goal_tolerance()}'
        ]
        output = '\n'.join(string_list)
        if print:
            rospy.loginfo(output)
        return output
    
    # reveals the latest action feedback of the last call to move_group
    def get_latest_moveit_feedback(self) -> MoveGroupActionFeedback:
        """Get the latest feedback resulting from the last command

        :return: The latest action feedback of the last call to the underlying move commander
        :rtype: MoveGroupActionFeedback
        """
        return self.cached_result

    # returns the name of the end effector link defined in move_group
    def get_end_effector_link(self) -> str:
        """ Return the name of the end effector link defined in move_group
        
        :return: The name of the end effector link
        :rtype: str
        """
        return self.END_EFFECTOR_LINK
    
    # returns the name of the world reference frame 
    def get_world_reference_frame(self) -> str:
        """ Returns the name of the world reference frame
        
        :return: The name of the world reference frame
        :rtype: str
        """
        return self.WORLD_REFERENCE_LINK
    
    # returns the current state (GeneralCommanderStates) of the commander
    def get_commander_state(self, print=False) -> GeneralCommanderStates:
        """Get the current state of this general commander

        :param print: Sends the state to the output as well, defaults to False
        :type print: bool, optional
        :return: The current state of this general commander
        :rtype: GeneralCommanderStates
        """
        if print:
            rospy.loginfo(self.commander_state.name)
        return self.commander_state
    
    # reveals the error code of the latest action of the move_group 
    def get_error_code(self, print=False) -> MoveItErrorCodes:
        """Get the error code of resulting from the last command

        :param print: Sends the error code to the output as well, defaults to False
        :type print: bool, optional
        :return: The error code of the last command
        :rtype: MoveItErrorCodes
        """
        if self.cached_result is None:
            return None
        if print:
            if self.cached_result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Move Group has no error')
            rospy.loginfo(f'Move Group Error: {MoveItErrorCodes(self.cached_result.result.error_code.val)}')
        return self.cached_result.result.error_code 

    # returns the positions of all the joints in a dict of (joint_name, position)
    def current_joint_positions(self, print=False) -> dict:
        """ Get the positions of all the joints

        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: Pairs of (joint_name, position) in a dictionary
        :rtype: dict
        """
        joint_values_dict = self.robot.get_current_variable_values()
        if print: rospy.loginfo(joint_values_dict)
        return joint_values_dict
    
    # returns the positions of the joints in a list
    def current_joint_positons_as_list(self, print=False) -> list:
        """ Get the positions of all the joints

        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: Lists of joint positions
        :rtype: list
        """
        joint_values = self.move_group.get_current_joint_values()
        if print: rospy.loginfo(joint_values)
        return joint_values        
    
    # returns the pose (as xyzq list of 7 floats) of a link (link_name) in a frame of reference (reference_frame)
    def pose_in_frame_as_xyzq(self, link_name:str=None, reference_frame:str=None, ros_time:rospy.Time=None, print=False) -> list:
        """ Get the pose of a link as a list of 7 floats (xyzq)

        :param link_name: The name of the link to be queried, defaults to the end-effector
        :type link_name: str, optional
        :param reference_frame: The name of the frame of reference, defaults to the world/default
        :type reference_frame: str, optional
        :param ros_time: The time when the pose is queried, defaults to current time
        :type ros_time: rospy.Time, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :raises ValueError: when the names of the link_name or reference_frame is invalid
        :return: The pose in the format of a list of 7 floats (xyzq)
        :rtype: list
        """
        link_name = self.END_EFFECTOR_LINK if link_name is None else link_name
        ros_time = rospy.Time(0) if ros_time is None else ros_time
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame
        try:
            self.tf_listener.waitForTransform(reference_frame, link_name, rospy.Time.now(), rospy.Duration(5.0))
            (trans, q) = self.tf_listener.lookupTransform(reference_frame, link_name, rospy.Time(0))
            if print: 
                if reference_frame is None:
                    reference_frame = self.WORLD_REFERENCE_LINK
        except (tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr(f'Invalid parameter or TF error: {e}')
            raise
        except tf.ExtrapolationException as e:
            rospy.logerr(f'Extrapolation error: {e}')
            raise
        return trans + q
    
    # returns the pose (as xyzrpy list of 6 floats) of a link (link_name) in a frame of reference (reference_frame)    
    def pose_in_frame_as_xyzrpy(self, link_name:str=None, reference_frame:str=None, ros_time:rospy.Time=None, print=False) -> list:
        """ Get the pose of a link as a list of 6 floats (xyzrpy)

        :param link_name: The name of the link to be queried, defaults to the end-effector
        :type link_name: str, optional
        :param reference_frame: The name of the frame of reference, defaults to the world/default
        :type reference_frame: str, optional
        :param ros_time: The time when the pose is queried, defaults to current time
        :type ros_time: rospy.Time, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :raises ValueError: when the names of the link_name or reference_frame is invalid
        :return: The pose in the format of a list of 6 floats (xyzrpy)
        :rtype: list
        """
        link_name = self.END_EFFECTOR_LINK if link_name is None else link_name
        pose = self.pose_in_frame_as_xyzq(link_name, reference_frame, ros_time)
        xyzrpy = pose[:3]
        xyzrpy.extend(tf.transformations.euler_from_quaternion(pose[3:]))
        if print: 
            if reference_frame is None:
                reference_frame = self.WORLD_REFERENCE_LINK
            rospy.loginfo(f'pose of {link_name} from {reference_frame}: {xyzrpy}')
        return xyzrpy
    
    # returns the pose (as PoseStamped) of a link (link_name) in a frame of reference (reference_frame)
    def pose_in_frame(self, link_name:str=None, reference_frame:str=None, ros_time:rospy.Time=None) -> PoseStamped:
        """ Get the pose of a link as a PoseStamped

        :param link_name: The name of the link to be queried, defaults to the end-effector
        :type link_name: str, optional
        :param reference_frame: The name of the frame of reference, defaults to the world/default
        :type reference_frame: str, optional
        :param ros_time: The time when the pose is queried, defaults to current time
        :type ros_time: rospy.Time, optional
        :raises ValueError: when the names of the link_name or reference_frame is invalid
        :return: The pose as a PoseStamped
        :rtype: PoseStamped
        """
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame        
        link_name = self.END_EFFECTOR_LINK if link_name is None else link_name
        xyzq = self.pose_in_frame_as_xyzq(link_name, reference_frame, ros_time)
        return conversions.list_to_pose_stamped(xyzq, reference_frame)
    
    # returns the pose (as Pose) of a link (link_name) of the default move_group planning frame
    def pose_of_robot_link(self, robot_link_name:str=None, print=False) -> PoseStamped:
        """ Get the pose of a link as a PoseStamped in the default move_group planning frame

        :param robot_link_name: The name of the link to be queried, defaults to the end-effector
        :type robot_link_name: str, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of a link as a PoseStamped in the default move_group planning frame
        :rtype: PoseStamped
        """
        if robot_link_name is None:
            robot_link_name = self.END_EFFECTOR_LINK
        current_pose = self.move_group.get_current_pose(robot_link_name)
        if print: rospy.loginfo(current_pose)
        return current_pose
    
    # returns the orientation (as rpy) of a link (link_name) of the default move_group planning frame    
    def rpy_of_robot_link(self, robot_link_name:str=None, print=False) -> list:
        """ Get the orientation of a link as a list of euler angles (rpy) in the default move_group planning frame

        :param robot_link_name: The name of the link to be queried, defaults to the end-effector
        :type robot_link_name: str, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of a link as a list of euler angles (rpy) 
        :rtype: list
        """
        if robot_link_name is None:
            robot_link_name = self.END_EFFECTOR_LINK
        current_rpy = self.move_group.get_current_rpy(robot_link_name)
        if print: rospy.loginfo(current_rpy)
        return current_rpy
    
    # returns the pose (as xyzrpy) of a link (link_name) of the default move_group planning frame  
    def xyzrpy_of_robot_link(self, robot_link=None, print=False) -> list:
        """ Get the pose of a link as a list of 6 floats (xyzrpy) in the default move_group planning frame

        :param robot_link_name: The name of the link to be queried, defaults to the end-effector
        :type robot_link_name: str, optional
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of a link as a list of 6 floats (xyzrpy)
        :rtype: list
        """
        if robot_link is None:
            robot_link = self.END_EFFECTOR_LINK
        current_pose = self.move_group.get_current_pose(robot_link) 
        xyzrpy = [
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z]
        rpy = self.move_group.get_current_rpy(self.END_EFFECTOR_LINK)
        xyzrpy.extend(rpy)
        if print: rospy.loginfo(xyzrpy)
        return xyzrpy
    
    # returns the tranformation (PoseStamped) of a pose (the_pose) to another frame of reference (to_frame)
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
            self.tf_listener.waitForTransform(to_frame, the_pose.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
            transformed_pose = self.tf_listener.transformPose(to_frame, the_pose)
        except tf.ExtrapolationException as e:
            rospy.logerr(f'Extrapolation exception received: {e}')
            raise
        return transformed_pose
    
    # --- Functions for changing the state and parameters of the Commander
    # call to spin this node 
    def spin(self, spin_in_thread:bool=True) -> None:
        """Starts the arm commander. If the parameter is True, the arm_commander is running in a new thread, 
        If otherwise, the commander is running in the caller's thread, blocking the call.

        :param spin_in_thread: create a new thread for the loop, defaults to True
        :type spin_in_thread: bool, optional
        """
        if spin_in_thread:
            self.the_thread = threading.Thread(target=rospy.spin, daemon=True)
            self.the_thread.start()
            return
        rospy.spin()

    # set the state of the commander to READY, remove records of previous commands and ready to accept new comamnds
    def reset_state(self) -> None:
        """Set the state of the commander to READY, remove records of previous commands and ready to accept new comamnds
        """
        self.commander_state = GeneralCommanderStates.READY
        self.cached_result = None 
    
    # call to abort the current command
    def abort_move(self, wait=True):
        """ Abort the current command

        :param wait: blocks the return of function call until the abort operation has finished, defaults to True
        :type wait: bool, optional
        """
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if wait:
            self.wait_for_busy_end()
        
    # a blocking call to wait until the current command has left the BUSY state
    def wait_for_busy_end(self) -> GeneralCommanderStates:
        """blocks the call return until the current command has left the BUSY state

        :return: The final state of the commander
        :rtype: GeneralCommanderStates
        """
        while True:
            if self.commander_state not in [GeneralCommanderStates.BUSY]:
                return self.commander_state
            rospy.sleep(0.05)

    # set the maximum speed of the end-effector motion
    def set_max_cartesian_speed(self, max_speed:float=None):
        """Set the maximum speed of the end-effector motion

        :param max_speed: The maximum speed in meters per seconds, defaults to the system default speed
        :type max_speed: float, optional
        """
        if max_speed is None:
            self.move_group.clear_max_cartesian_link_speed()
        else:
            self.move_group.limit_max_cartesian_link_speed(max_speed, self.END_EFFECTOR_LINK)
    
    # set the maximum planning time (in seconds)
    def set_planning_time(self, planning_time:int):
        """Set the maximum planning time

        :param planning_time: The maximum planning time in seconds
        :type planning_time: int
        """
        self.move_group.set_planning_time(planning_time)
    
    # set the goal tolerance (a list of joint, position and orientation goal tolerances)
    def set_goal_tolerance(self, tol_list:list): 
        """Set the goal tolerance 

        :param tol_list: set the goal tolerance as a list of joint, position and orientation goal tolerances
        :type tol_list: list
        """
        self.move_group.set_goal_tolerance(tol_list) 
    
    # return a list of current joint, position and orientation goal tolerances
    def get_goal_tolerance(self) -> list:
        """ Get the current goal tolerance

        :return: A list of current joint, position and orientation goal tolerances
        :rtype: list
        """
        return self.move_group.get_goal_tolerance()

    # set the joint positions (as a list) of a named pose
    def add_named_pose(self, name: str, joint_values: list):
        """ Add a named pose specified in joint positions to the commander

        :param name: The name of the pose to be defined
        :type name: str
        :param joint_values: a list of joint values
        :type joint_values: list
        """
        self.move_group.remember_joint_values(name, values=joint_values)
    
    # remove the named pose from the commander
    def forget_named_pose(self, name: str):
        """Remove the named pose from the commander

        :param name: The name of the pose to be defined
        :type name: str
        """
        self.move_group.forget_joint_values(name)

    # runs a provided service given a namespace (ns) and input class (cls)
    def invoke_service(self, ns: str, cls, **kwargs):
        """Invokes a service of given namespace (ns) and class (cls)
        Credit: https://github.com/ros-controls/ros_control/issues/511 

        :param ns: Namespace of service to invoke
        :type ns: str
        :param cls: Class of service object
        :type cls: Any (depends on invoked class type. Check service in commandline using 'rosservice info ns')
        """
        rospy.wait_for_service(ns)
        service = rospy.ServiceProxy(ns, cls)
        response = service(**kwargs)
        if not response.ok:
            rospy.logwarn(f"[GeneralCommander::invoke_service][Running service failed...]")
        else:
            rospy.loginfo(f"[GeneralCommander::invoke_service][Service invoked successfully]")


    def controller_select(self, controller_type: ControllerState = ControllerState.TRAJECTORY) -> bool:
        """Attempts to select a given controller via controller manager

        :param controller_type: Controller state to set to, defaults to ControllerState.TRAJECTORY (0)
        :type controller_type: ControllerState, optional
        :return: Success (True) or Failure (False)
        :rtype: bool
        """
        start_controller = None
        stop_controller = None        
        if controller_type == ControllerState.TRAJECTORY:
            start_controller = 'trajectory_controller'
            stop_controller = 'servo_controller'
        elif controller_type == ControllerState.SERVO:
            start_controller = 'servo_controller'
            stop_controller = 'trajectory_controller'
        else:
            rospy.logerr(f"[GenearalCommander::controller_select][Invalid controller type: {controller_type}]")
            return False

        try:
            self.invoke_service("/controller_manager/switch_controller",
                SwitchController,
                start_controllers=[start_controller],
                stop_controllers=[stop_controller],
                strictness=1, start_asap=False, timeout=0.0)               
        except rospy.ServiceException as e:
            rospy.logerr(f"[GeneralCommander::controller_select][Could not switch controllers: {e}]")
            return False
        
        return True

    # --- Functions for sending a command of robot manipulation

    # command the robot arm to move to a named pose (joint-space) as defined in the scene model 
    def move_to_named_pose(self, named_pose:str, wait=True):
        """ Command the robot arm to move to a named pose (joint-space) as defined in the scene model or
        the commander through :method: 'add_named_pose'.

        :param named_pose: The name of pre-defined named pose
        :type named_pose: str
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        """
        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr('The commander (move_to_named_pose): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            self.move_group.set_named_target(named_pose)
            self.commander_state = GeneralCommanderStates.BUSY        
            success = self.move_group.go(wait=wait)
            if not wait:
                return         
            self.abort_move()
        except Exception as e:
            rospy.logerr(f'The commander (move_to_named_pose): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()

    # command the robot arm to move the end effector by a displacement (dx, dy, dz) in a cartesian motion manner
    def move_displacement(self, dx=0.0, dy=0.0, dz=0.0, accept_fraction=0.9999, wait=True) -> Pose:
        """Command the robot arm to move the end effector by a displacement (dx, dy, dz) 
        in a cartesian motion manner

        :param dx: displacement in x axis, defaults to 0
        :type dx: float, optional
        :param dy: displacement in y axis, defaults to 0
        :type dy: float, optional
        :param dz: displacement in z axis, defaults to 0
        :type dz: float, optional
        :param accept_fraction: the acceptable minimum fraction of the planned path towards the target or abort the command, defaults to 0.9999
        :type accept_fraction: float, optional
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose
        :rtype: Pose
        """
        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr('The commander (move_displacement): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            current_pose = self.move_group.get_current_pose(self.END_EFFECTOR_LINK)
            target_pose = copy.deepcopy(current_pose)
            target_pose.pose.position.x += dx
            target_pose.pose.position.y += dy
            target_pose.pose.position.z += dz
            waypoints = [current_pose.pose, target_pose.pose]
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, self.CARTE_PLANNING_STEP_SIZE, 0.00)
            if fraction < accept_fraction:
                rospy.logerr(f'Planning failed')
                self.commander_state = GeneralCommanderStates.ABORTED
                self.commander_state.message = 'PLANNING_FAILED_DUE_TO_COLLISION'
                return None 
            self.commander_state = GeneralCommanderStates.BUSY
            self.move_group.execute(plan, wait=wait)            
            if not wait:
                return target_pose
            self.abort_move()
            return target_pose
        except Exception as e:
            rospy.logerr(f'The commander (move_displacement): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'           
        finally:
            self.action_lock.release()

    # command the robot arm to move the end effector to a position (x, y, z) optionally in a cartesian motion manner and 
    # optionally in a particular frame of reference
    def move_to_position(self, x:float=None, y:float=None, z:float=None, accept_fraction:float=0.9999, cartesian=False, 
                         reference_frame:str=None, wait=True) -> Pose:
        """Command the robot arm to move the end effector to a position (x, y, z) optionally 
        in a cartesian motion manner and optionally in a specified frame of reference

        :param x: the target x position, defaults to the current x position
        :type x: float, optional
        :param y: the target y position, defaults to the current y position
        :type y: float, optional
        :param z: the target z position, defaults to the current z position
        :type z: float, optional
        :param accept_fraction: the acceptable minimum fraction of the planned path towards the target or abort the command, defaults to 0.9999
        :type accept_fraction: float, optional
        :param cartesian: use cartesian motion as the path, defaults to False
        :type cartesian: bool, optional
        :param reference_frame: frame of reference in which positions are specified, defaults to the world/default
        :type reference_frame: str, optional
        :param wait: the call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose in the given reference frame
        :rtype: Pose
        """

        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr('The commander (move_to_position): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
            target_pose:PoseStamped = self.pose_in_frame(self.END_EFFECTOR_LINK, reference_frame)
            target_pose.pose.position.x = x if x is not None else target_pose.pose.position.x
            target_pose.pose.position.y = y if y is not None else target_pose.pose.position.y
            target_pose.pose.position.z = z if z is not None else target_pose.pose.position.z
            if cartesian:
                current_in_world_frame:PoseStamped = self.pose_in_frame(self.END_EFFECTOR_LINK, self.WORLD_REFERENCE_LINK)
                target_in_world_frame:PoseStamped = self.transform_pose(target_pose, self.WORLD_REFERENCE_LINK)
                waypoints = [current_in_world_frame.pose, target_in_world_frame.pose]
                self.commander_state = GeneralCommanderStates.BUSY
                (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, self.CARTE_PLANNING_STEP_SIZE, 0.00) 
                if fraction < accept_fraction:
                    rospy.logerr(f'The commander (move_to_position): planning failed due to collision ({fraction})')
                    self.commander_state, self.commander_state.message = GeneralCommanderStates.ABORTED, 'PLANNING_FAILED_DUE_TO_COLLISION'
                    return None 
                self.move_group.execute(plan, wait=wait)
            else:
                self.move_group.set_pose_target(target_pose)
                self.commander_state = GeneralCommanderStates.BUSY
                self.move_group.go(wait=wait)
            if not wait:
                return target_pose
            self.abort_move()
            return target_pose 
        except Exception as e:
            rospy.logerr(f'The commander (move_to_position): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()


    # command the robot arm to move the end effector to a series of positions defined in a list of 3-tuple (x, y, z)
    # or 3-list [x, y, z]  in a cartesian motion manner and optionally in a particular frame of reference
    def move_to_multi_positions(self, xyz_list, reference_frame:str=None, wait=True) -> Pose:
        """Command the robot arm to move the end effector to a series of positions defined in a list of 3-tuple (x, y, z)
        or 3-list [x, y, z] in a cartesian motion manner and optionally in a specified frame of reference. The xyz tuple or list
        can contain a None value, which is default to the respective value of the current pose (at the start).

        :param xyz_list: the waypoint and target positions, defaults to the current x position
        :type xyz_list: list of 3-tuple or 3-list
        :param reference_frame: frame of reference in which positions are specified, defaults to the world/default
        :type reference_frame: str, optional
        :param wait: the call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose in the given reference frame
        :rtype: Pose
        """

        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr('The commander (move_to_multi_positions): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            if xyz_list is None or type(xyz_list) not in [list, tuple]:
                rospy.logerr('The parameter xyz_list (move_to_multi_positions) is not a list or tuple')
                return                 
            reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
            current_in_world_frame:PoseStamped = self.pose_in_frame(self.END_EFFECTOR_LINK, self.WORLD_REFERENCE_LINK)
            waypoints_list = [current_in_world_frame.pose]
            for xyz in xyz_list:
                if xyz is None or type(xyz) not in [list, tuple] or len(xyz) != 3 \
                        or any((n is not None) and (not isinstance(n, numbers.Number)) for n in xyz):
                    rospy.logerr('The parameter xyz_list (move_to_multi_positions) contains waypoints that is not a list of 3 numbers')
                    return                 
                waypoint_pose:PoseStamped = self.pose_in_frame(self.END_EFFECTOR_LINK, reference_frame)
                waypoint_pose.pose.position.x = xyz[0] if xyz[0] is not None else waypoint_pose.pose.position.x
                waypoint_pose.pose.position.y = xyz[1] if xyz[1] is not None else waypoint_pose.pose.position.y
                waypoint_pose.pose.position.z = xyz[2] if xyz[2] is not None else waypoint_pose.pose.position.z
                waypoint_in_world_frame:PoseStamped = self.transform_pose(waypoint_pose, self.WORLD_REFERENCE_LINK)
                waypoints_list.append(waypoint_in_world_frame.pose)

            self.commander_state = GeneralCommanderStates.BUSY
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints_list, self.CARTE_PLANNING_STEP_SIZE, 0.00) 
            if fraction < 0.999:
                rospy.logerr(f'The commander (move_to_multi_positions): planning failed due to collision ({fraction})')
                self.commander_state, self.commander_state.message = GeneralCommanderStates.ABORTED, 'PLANNING_FAILED_DUE_TO_COLLISION'
                return None 
            self.move_group.execute(plan, wait=wait)
            if not wait:
                return waypoints_list
            self.abort_move()
            return waypoints_list 
        except Exception as e:
            rospy.logerr(f'The commander (move_to_multi_positions): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()

    # command the robot arm to rotate the end effector to an orientation (rpy) optionally in a particular frame of reference
    def rotate_to_orientation(self, roll:float=None, pitch:float=None, yaw:float=None, reference_frame:str=None, wait=True):
        """Command the robot arm to rotate the end effector to an orientation (rpy) optionally in a particular frame of reference
  
        :param roll: The target roll orientation, defaults to the current value 
        :type roll: float, optional
        :param pitch: The target pitch orientation, defaults to the current value
        :type pitch: float, optional
        :param yaw: The target yaw orientation, defaults to the current value
        :type yaw: float, optional
        :param reference_frame: The frame of reference in which positions are specified, defaults to the world/default
        :type reference_frame: str, optional
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose in the given reference frame
        :rtype: Pose
        """
        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr(f'{__class__.__name__} (rotate_to_orientation): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
            target_xyzrpy = self.pose_in_frame_as_xyzrpy(self.END_EFFECTOR_LINK, reference_frame=reference_frame)
            target_xyzrpy[3] = roll if roll is not None else target_xyzrpy[3]
            target_xyzrpy[4] = pitch if pitch is not None else target_xyzrpy[4]
            target_xyzrpy[5] = yaw if yaw is not None else target_xyzrpy[5]
            target_pose = conversions.list_to_pose_stamped(target_xyzrpy, reference_frame)
            self.move_group.set_pose_target(target_pose)
            # rospy.loginfo(f'target_pose: {target_xyzrpy} {target_pose}')
            self.commander_state = GeneralCommanderStates.BUSY
            success = self.move_group.go(wait=wait)
            if not wait:
                return target_pose
            self.abort_move()
            return target_pose
        except Exception as e:
            rospy.logerr(f'{__class__.__name__} (rotate_to_orientation): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()
    
    # command the robot arm to move the end effector to a pose optionally in a particular frame of reference   
    # the target_pose may be a list of 6 or 7 numbers (xyz+4q) or (xyz+rpy) or Pose or PoseStamped  
    def move_to_pose(self, target_pose, reference_frame:str=None, wait=True) -> Pose:
        """Command the robot arm to move the end effector to a pose optionally in a particular frame of reference   
            the target_pose may be a list of 6 numbers (xyz+rpy), 7 numbers (xyz+4q), Pose or PoseStamped 
            
        :param target_pose: The target pose
        :type target_pose: a list of 6 floats (xyzrpy), 7 floats (xyzq), Pose, or PoseStamped
        :param reference_frame: The frame of reference in which positions are specified, defaults to the world/default
        :type reference_frame: str, optional
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose in the given reference frame
        :rtype: Pose
        """
        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr(f'{__class__.__name__} (move_to_pose): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
            if type(target_pose) is list: 
                target_pose = conversions.list_to_pose_stamped(target_pose, reference_frame)
            self.move_group.set_pose_target(target_pose)
            self.commander_state = GeneralCommanderStates.BUSY
            self.move_group.go(wait=wait)
            if not wait:
                return target_pose
            self.abort_move()
            return target_pose

        except Exception as e:
            rospy.logerr(f'{__class__.__name__} (move_to_pose): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()

    # command the robot arm to move the end effector to a series of (waypoint) poses optionally in a particular frame of reference   
    # each waypoint pose may be a list of 6 or 7 numbers (xyz+4q) or (xyz+rpy) or Pose or PoseStamped. No None value is acceptable
    def move_to_multi_poses(self, waypoints_list, reference_frame:str=None, wait=True) -> Pose:
        """Command the robot arm to move the end effector to a series of (waypoint) poses optionally in a particular frame of reference 
            the waypoint pose may be a list of 6 numbers (xyz+rpy), 7 numbers (xyz+4q), Pose or PoseStamped 
            
        :param waypoints_list: The list of waypoint pose
        :type waypoints_list: a list of waypoint poses, each of which is a list of 6 floats (xyzrpy), 7 floats (xyzq), Pose, or PoseStamped
        :param reference_frame: The frame of reference in which positions are specified, defaults to the world/default
        :type reference_frame: str, optional
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        :return: The target pose in the given reference frame
        :rtype: Pose
        """
        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr(f'{__class__.__name__} (move_to_multi_poses): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            if waypoints_list is None or type(waypoints_list) not in [list, tuple]:
                rospy.logerr('The parameter waypoints_list (move_to_multi_poses) is not a list or tuple')
                return                 
            reference_frame = reference_frame if reference_frame is not None else self.WORLD_REFERENCE_LINK
            current_in_world_frame:PoseStamped = self.pose_in_frame(self.END_EFFECTOR_LINK, self.WORLD_REFERENCE_LINK)            
            processed_waypoints_list = [current_in_world_frame.pose]
            for waypoint in waypoints_list:
                if type(waypoint) in [list, tuple]: 
                    waypoint = conversions.list_to_pose_stamped(waypoint, reference_frame)
                if type(waypoint) == Pose:
                    converted = PoseStamped()
                    converted.pose = waypoint
                    converted.header.frame_id = reference_frame
                    waypoint = converted
                if type(waypoint) != PoseStamped:
                    rospy.logerr(f'The parameter waypoints_list (move_to_multi_poses) contains a waypoint of invalid format: {waypoint}')
                    return  
                waypoint_in_world_frame:PoseStamped = self.transform_pose(waypoint, self.WORLD_REFERENCE_LINK)
                processed_waypoints_list.append(waypoint_in_world_frame.pose)

            self.commander_state = GeneralCommanderStates.BUSY
            (plan, fraction) = self.move_group.compute_cartesian_path(processed_waypoints_list, self.CARTE_PLANNING_STEP_SIZE, 0.00) 
            if fraction < 0.999:
                rospy.logerr(f'The commander (move_to_multi_poses): planning failed due to collision ({fraction})')
                self.commander_state, self.commander_state.message = GeneralCommanderStates.ABORTED, 'PLANNING_FAILED_DUE_TO_COLLISION'
                return None 
            self.move_group.execute(plan, wait=wait)
                                   
            if not wait:
                return processed_waypoints_list
            self.abort_move()
            return processed_waypoints_list

        except Exception as e:
            rospy.logerr(f'{__class__.__name__} (move_to_multi_poses): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()

    # command the robot arm to move to a pose defined by joint values
    def move_to_joint_pose(self, joint_pose:list, wait=True):
        """ Command the robot arm to move to a named pose (joint-space) as defined in the scene model or
        the commander through :method: 'move_to_joint_pose'.

        :param joint_pose: The list of joint value
        :type joint_pose: list
        :param wait: The call is blocked until the command has been completed, defaults to True
        :type wait: bool, optional
        """
        self.action_lock.acquire()
        try:
            if self.commander_state != GeneralCommanderStates.READY:
                rospy.logerr('The commander (move_to_joint_pose): to move while not READY state -> ensure in the client the previous command is completed and commander reset')
                return
            self.move_group.set_joint_value_target(joint_pose)
            self.commander_state = GeneralCommanderStates.BUSY        
            success = self.move_group.go(wait=wait)
            if not wait:
                return         
            self.abort_move()
        except Exception as e:
            rospy.logerr(f'The commander (move_to_joint_pose): {e} {traceback.format_exc()}')
            self.commander_state = GeneralCommanderStates.ABORTED
            self.commander_state.message = 'MOVE_FAILED_DUE_TO_EXCEPTION'  
        finally:
            self.action_lock.release()

    def servo_robot(self, twist: Twist, frame: str = "", time: float = 0) -> bool:
        """Attempt to servo the robot with provided Twist

        :param twist: Twist message containing linear and angular velocity components
        :type twist: Twist
        :param frame: Frame to apply twist to (from robot tree)
        :type frame: str
        :param time: Time in seconds to apply twist, defaults to 0 seconds
        :type time: float, optional
        :return: True on success, else False
        :rtype: bool
        """
        
        # Error checking on empty servo
        if not isinstance(twist, Twist):
            rospy.logerr(f'[GeneralCommander::servo_robot][Invalid twist provided. Exiting]')
            return False
        
        # Handle empty frame input -> default to base_frame
        if frame == "" or frame == None:
            rospy.logwarn(f'[GeneralCommander::servo_robot][Defaulting to {self.WORLD_REFERENCE_LINK}]')
            frame = self.WORLD_REFERENCE_LINK

        # Set timeout parameters
        ros_rate = rospy.Rate(100) #hz
        default_timeout = timeit.default_timer()

        # Construct twiststamped
        twist_stamped = TwistStamped()
        twist_stamped.header.frame_id = frame
        twist_stamped.twist = twist

        # Set to timeout on input time (seconds)
        while not rospy.is_shutdown() and timeit.default_timer() - default_timeout < time:
            # Update time header
            twist_stamped.header.stamp = rospy.Time.now()

            # Publish to Moveit Servo Server
            self.servo_pub.publish(twist_stamped)

            # Sleep
            ros_rate.sleep()

        # Send stop twist (empty)
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.twist = Twist()
        self.servo_pub.publish(twist_stamped)    

        return True

    # -- Functions: objects for collision detection and planning
    
    # remove all scene objects that have been added through the commander
    def reset_world(self):
        """Remove all scene objects that have been added through the commander
        """
        
        self.move_group.detach_object(self.END_EFFECTOR_LINK)
        self.scene.remove_attached_object(self.END_EFFECTOR_LINK)
        self.scene.remove_world_object()
    
    # wait for the scene to update after addition or removal of collision objects
    def _wait_for_scene_update(self, func, timeout=5.0):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < timeout):
            if func():
                return
            rospy.sleep(0.2)
        rospy.logwarn(f'{__class__.__name__} (_wait_for_scene_update): timeout ({timeout} seconds)')

    # # add an object (defined with a meshed model, pose, scale) to the scene
    # def add_object_to_scene(self, object_name:str, model_file:str, object_scale:list, xyz:list, rpy:list):
    #     """Add an object (defined with a meshed model, pose, scale) to the scene for collision avoidance in path planning

    #     :param object_name: The name given to the new scene object
    #     :type object_name: str
    #     :param model_file: The path to the file that defines the mesh of the object
    #     :type model_file: str
    #     :param object_scale: The list of scale in 3 dimension
    #     :type object_scale: list
    #     :param xyz: The position of the object in the world/default reference frame
    #     :type xyz: list
    #     :param rpy: The orientation of the object in euler angles in the world/default reference frame
    #     :type rpy: list
    #     """
    #     self.scene.remove_world_object(object_name)
    #     object_pose = conversions.list_to_pose_stamped(xyz + rpy, self.WORLD_REFERENCE_LINK)
    #     self.scene.add_mesh(
    #             object_name, object_pose,
    #             model_file, object_scale
    #     )
    #     self._wait_for_scene_update(lambda: object_name in self.scene.get_known_object_names())
    #     self._pub_transform_object(object_name, object_pose)
    
    # add an object (defined with a meshed model, pose, scale) to the scene
    def add_object_to_scene(self, object_name:str, model_file:str, object_scale:list, xyz:list, rpy:list, reference_frame:str=None):
        """Add an object (defined with a meshed model, pose, scale) to the scene for collision avoidance in path planning

        :param object_name: The name given to the new scene object
        :type object_name: str
        :param model_file: The path to the file that defines the mesh of the object
        :type model_file: str
        :param object_scale: The list of scale in 3 dimension
        :type object_scale: list
        :param xyz: The position of the object in the world/default reference frame
        :type xyz: list
        :param rpy: The orientation of the object in euler angles in the world/default reference frame
        :type rpy: list
        :param reference_frame: The frame of reference of the xyz and rpy
        :type reference_frame: str, default to WORLD_REFERENCE_LINK        
        """
        
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame
        self.scene.remove_world_object(object_name)
        object_pose = conversions.list_to_pose_stamped(xyz + rpy, reference_frame)
        self.scene.add_mesh(
                object_name, object_pose,
                model_file, object_scale
        )
        self._wait_for_scene_update(lambda: object_name in self.scene.get_known_object_names())
        self._pub_transform_object(object_name, object_pose)
    
    # add an object (a shpere of given radius and position)
    def add_sphere_to_scene(self, object_name:str, radius:float, xyz:list, reference_frame:str=None):
        """Add an object to the scene for collision avoidance in path planning

        :param object_name: The name given to the new scene object
        :type object_name: str
        :param radius: The radius of the sphere
        :type radius: float
        :param xyz: The position of the sphere in the world/default reference frame
        :type xyz: list
        :param reference_frame: The frame of reference of the xyz and rpy
        :type reference_frame: str, default to WORLD_REFERENCE_LINK     
        """
        
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame
        object_pose = conversions.list_to_pose_stamped(xyz + [0, 0, 0], reference_frame)
        self.scene.add_sphere(object_name, object_pose, radius)
        self._wait_for_scene_update(lambda: object_name in self.scene.get_known_object_names())
        self._pub_transform_object(object_name, object_pose) 
        
    # add a box (a box of given dimension, position and orientation)    
    def add_box_to_scene(self, object_name:str, dimensions:list, xyz:list, rpy:list=[0, 0, 0], reference_frame:str=None):
        """ Add a box to the scene for collision avoidance in path planning
        
        :param object_name: The name given to the new scene object
        :type object_name: str
        :param dimensions: The dimensions of the box as a list of 3 floats
        :type radius: list
        :param xyz: The position of the box in the world/default reference frame
        :type xyz: list
        :param rpy: The orientation of the box in the world/default reference frame
        :type rpy: list
        :param reference_frame: The frame of reference of the xyz and rpy
        :type reference_frame: str, default to WORLD_REFERENCE_LINK     
        """
        
        reference_frame = self.WORLD_REFERENCE_LINK if reference_frame is None else reference_frame
        object_pose = conversions.list_to_pose_stamped(xyz + rpy, reference_frame)
        self.scene.add_box(object_name, object_pose, size=dimensions)
        self._wait_for_scene_update(lambda: object_name in self.scene.get_known_object_names())
        self._pub_transform_object(object_name, object_pose)      
        
    # returns a list of current objects that have been added to the commander
        """ Returns a list of current objects that have been added to the commander
        """
        
    def list_object_names(self) -> list:
        return self.scene.get_known_object_names()
    
    # remove object given the name
    def remove_object(self, object_name:str) -> None:
        """ Remove object given the name or all objects in the scene if None

        :param object_name: The name of the object, None for removing all objects
        :type object_name: str
        """
        
        self.scene.remove_world_object(object_name)
        self._wait_for_scene_update(lambda: object_name not in self.scene.get_known_object_names())

    # attach an object to the end_effector
    def attach_object_to_end_effector(self, object_name:str) -> bool:
        """ Attach an object, name defined in the world, to the end effector 
        :param object_name: the name of the object to be attached
        :type object_name: str
        :return: True if the attach is successful
        :rtype: bool
        """
        
        result = self.move_group.attach_object(object_name, link_name=self.END_EFFECTOR_LINK)
        self._wait_for_scene_update(lambda: len(self.scene.get_attached_objects([object_name]).keys()) > 0)
        return result

    # detach the object given the name from the robot
    def detach_object_from_end_effector(self, object_name:str) -> None:
        """ detach the named object from the robot
        :param object_name: the name of the object to be detached
        :type object_name: str
        """
        
        self.move_group.detach_object(object_name)
        # self.scene.remove_attached_object(self.END_EFFECTOR_LINK, name=object_name)
        self._wait_for_scene_update(lambda: len(self.scene.get_attached_objects([object_name]).keys()) == 0)        

    # detach all attached objects from the end_effector
    def detach_all_from_end_effector(self) -> None:
        """ detach all attached objects from the end_effector
        """
        
        self.move_group.detach_object(self.END_EFFECTOR_LINK) 
        # self.scene.remove_attached_object(self.END_EFFECTOR_LINK)
        self._wait_for_scene_update(lambda: len(self.scene.get_attached_objects([]).keys()) == 0) 

    # returns the pose (as Pose) of the object given its name
    def get_object_pose(self, object_name:str, print=False) -> Pose:
        """Returns the pose of the object given its name

        :param object_name: The name of the queried object
        :type object_name: str
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of the queried object
        :rtype: Pose
        """
        try:
            pose = self.scene.get_object_poses([object_name])
            pose = pose[object_name]
            if print:
                rospy.loginfo(f'Object {object_name} pose: {pose}')
            return pose
        except:
            rospy.logerr(f'{__class__.__name__} (get_object_pose): The value "{object_name}" parameter (object_name) is invalid or undefined -> fix the parameter')
            raise ValueError(f'Invalid parameter value')
        
    # returns the pose (as xyzrpy) of the object given its name    
    def get_object_pose_as_xyzrpy(self, object_name:str, print=False) -> list:
        """Returns the pose of the object given its name

        :param object_name: The name of the queried object
        :type object_name: str
        :param print: Sends the information to the output as well, defaults to False
        :type print: bool, optional
        :return: The pose of the queried object as a list of 6 floats (xyzrpy)
        :rtype: list
        """
        
        object_pose = self.get_object_pose(object_name)
        object_pose_as_list = conversions.pose_to_list(object_pose)
        xyzrpy = object_pose_as_list[:3]
        xyzrpy.extend(tf.transformations.euler_from_quaternion(object_pose_as_list[3:]))
        if print:
            rospy.loginfo(xyzrpy)
        return xyzrpy
        
    # sets up the limits of the workspace using 6 walls
    def set_workspace_walls(self, min_x:float, min_y:float, min_z:float, max_x:float, max_y:float, max_z:float):
        """A convenient function for setting up a workspace as a space surrounded by 6 walls

        :param min_x: The minimum x of the workspace
        :type min_x: float
        :param min_y: The minimum y of the workspace
        :type min_y: float
        :param min_z: The minimum z of the workspace
        :type min_z: float
        :param max_x: The maximum x of the workspace
        :type max_x: float
        :param max_y: The maximum y of the workspace
        :type max_y: float
        :param max_z: The maximum z of the workspace
        :type max_z: flost
        """
        thickness = 0.02
        size_x = max_x - min_x
        size_y = max_y - min_y
        size_z = max_z - min_z
        cx = (max_x + min_x) / 2
        cy = (max_y + min_y) / 2
        cz = (max_z + min_z) / 2
        self._create_wall('ws_top', self.WORLD_REFERENCE_LINK, cx, cy, max_z, size_x, size_y, thickness)
        self._create_wall('ws_bottom', self.WORLD_REFERENCE_LINK, cx, cy, min_z, size_x, size_y, thickness)  
        self._create_wall('ws_side_1', self.WORLD_REFERENCE_LINK, cx, min_y, cz, size_x, thickness, size_z)
        self._create_wall('ws_side_2', self.WORLD_REFERENCE_LINK, cx, max_y, cz, size_x, thickness, size_z)  
        self._create_wall('ws_side_3', self.WORLD_REFERENCE_LINK, min_x, cy, cz, thickness, size_y, size_z)
        self._create_wall('ws_side_4', self.WORLD_REFERENCE_LINK, max_x, cy, cz, thickness, size_y, size_z) 
        rospy.sleep(1.0) # wait for the walls to be registered
        self._pub_workspace_color()

    # internal function for creating a wall
    def _create_wall(self, name, frame_id, x, y, z, size_x, size_y, size_z):
        self.scene.remove_world_object(name)
        box_pose = PoseStamped()
        box_pose.header.frame_id = frame_id
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        self.scene.add_box(name, box_pose, size=(size_x, size_y, size_z))
        if name not in self.wall_name_list:
            self.wall_name_list.append(name)

    # internal function for setting the colour of a wall
    def _set_object_color(self, name, r, g, b, a = 0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        return color

    # --------------------------------
    # functions for setting path constraints
      
    def add_path_constraints(self, constraint):
        """Add a constraint to the commander. Refer to :module:'arm_commander.moveit_tools' for functions to conveniently create
        a constraint object 

        :param constraint: The constraint to be added to the commander
        :type constraint: OrientationConstraint, JointConstraint or PositionConstraint
        """
        
        the_type = type(constraint)
        if the_type == OrientationConstraint:
            self.the_constraints.orientation_constraints.append(constraint)
        elif the_type == JointConstraint:
            self.the_constraints.joint_constraints.append(constraint)
        elif the_type == PositionConstraint:
            self.the_constraints.position_constraints.append(constraint)           
        self.move_group.set_path_constraints(self.the_constraints)
        
    def clear_path_constraints(self):
        """Clear all the added constraints in the commander
        """
        
        self.the_constraints.orientation_constraints.clear()
        self.the_constraints.joint_constraints.clear()
        self.the_constraints.position_constraints.clear()
        self.move_group.set_path_constraints(None)
    
    def disable_path_constraints(self):
        """Disable the added constraints from taking effect in the next commands
        """
        
        self.move_group.set_path_constraints(None)
        
    def enable_path_constraints(self):
        """Enable the added constraints
        """
        
        self.move_group.set_path_constraints(self.the_constraints)

# ------------------------------------------------------------------------------------
# the factory class for creating and managing singleton agents
# that one agent object is to be shared between the users

class GeneralCommanderFactory():
    """ The factory object ensures that, in a system involving multiple general commander object, there is at most
    one object for each manipulator
    """
    object_store = dict()
    def get_object(moveit_group_name:str, world_link:str=None) -> GeneralCommander:
        """ Returns, and if needed, creates an general commander for the given moveit_group_name, 
        a manipulation group defined in the setup assistant (or the custom.env file)

        :param moveit_group_name: The name of the manipulation group
        :type moveit_group_name: str
        :param world_link: the frame of reference for planning, defaults to the world defined in the moveit configuraiton
        :type world_link: str, optional
        :return: The general commander for the manipulation group
        :rtype: GeneralCommander
        """
        if moveit_group_name in GeneralCommanderFactory.object_store:
            return GeneralCommanderFactory.object_store[moveit_group_name]
        the_commander = GeneralCommander(moveit_group_name, world_link)
        GeneralCommanderFactory.object_store[moveit_group_name] = the_commander
        return the_commander
    
# ----------------------------------------------------------------------------------
# the logging tool for use by the client program
#
    
class CustomFormatter(logging.Formatter):
    """
    :meta private:
    """
    grey = '\x1b[38;20m'
    cyan ='\x1b[36;20m'
    yellow = '\x1b[33;20m'
    red = '\x1b[31;20m'
    bold_red = '\x1b[31;1m'
    reset = '\x1b[0m'
    # format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)'
    format = '[%(levelname)s] [%(created)16f]: %(message)s'
    FORMATS = {
        logging.DEBUG: cyan + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }
    def format(self, record):
        time_format = "%H:%M:%S %f"
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt=time_format)
        return formatter.format(record)
    def formatException(self, exc_info):
        result = super().formatException(exc_info)
        return repr(result)

def init_logger():
    logger = logging.getLogger('main')
    logger.setLevel(logging.INFO)
    logger.propagate = False
    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setFormatter(CustomFormatter())
        logger.addHandler(ch)
    return logger

logger = init_logger()