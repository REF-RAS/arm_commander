#!/usr/bin/env python3
# Copyright 2024 - Andrew Kwok Fai LUI
# Robotics and Autonomous Systems Group
# Research Engineering Facility, Queensland University of Technology (QUT)
#
# License: https://github.com/REF-RAS/arm_commander/blob/main/LICENSE

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.1.0'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

import sys, math

import moveit_commander.conversions as conversions
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint, PositionConstraint, BoundingVolume
from moveit_msgs.msg import MoveItErrorCodes
from actionlib_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
import tf, tf.transformations

def same_pose_with_tolerence(pose_1, pose_2, tolerance:float) -> bool:
    """ Test if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        Applicable for joint space.  
        
    :param goal: A pose
    :type goal:  A list of floats, a Pose or a PoseStamped
    :param actual: another pose
    :type actual:  A list of floats, a Pose or a PoseStamped
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the goal and actual are the same 
    :rtype: bool
    """   
    if type(pose_1) is PoseStamped:
        return same_pose_with_tolerence(pose_1.pose, pose_2.pose, tolerance)
    elif type(pose_1) is Pose:
        xyzq0 = conversions.pose_to_list(pose_2)
        xyzq1 = conversions.pose_to_list(pose_1)
        d = math.dist((xyzq1[0], xyzq1[1], xyzq1[2]), (xyzq0[0], xyzq0[1], xyzq0[2])) 
        cos_phi_half = math.fabs(xyzq0[3] * xyzq1[3] + xyzq0[4] * xyzq1[4] + xyzq0[5] * xyzq1[5] + xyzq0[6] * xyzq1[6]) 
        return d <= tolerance and cos_phi_half >= math.cos(tolerance / 2.0) 
    else:
        if type(pose_1) is list:
            pose_1 = conversions.list_to_pose(pose_1)
        if type(pose_2) is list:
            pose_2 = conversions.list_to_pose(pose_2)  
        if type(pose_1) is not Pose or type(pose_2) is not Pose:
            print(f'same_pose_with_tolerence: expects pose as a list, Pose, PoseStamped -> fix the missing value at the function call')
            raise AssertionError(f'A parameter is invalid')
        return same_pose_with_tolerence(pose_1, pose_2, tolerance)

def same_joint_values_with_tolerence(joint_values_1:list, joint_values_2:list, tolerance:float) -> bool:
    """ Test if the two sets of joint values are the same

    :param joint_values_1: the first set of joint_values
    :type joint_values_1: list
    :param joint_values_2: the second set of joint values
    :type joint_values_2: list
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the two sets of joint values are deemed the same
    :rtype: bool
    """
    if len(joint_values_1) != len(joint_values_2):
        return False
    for index in range(len(joint_values_1)):
        if abs(joint_values_1[index] - joint_values_2[index]) > tolerance:
            return False
    return True

def same_rpy_with_tolerence(rpy_1:list, rpy_2:list, tolerance:float) -> bool:
    """ Test if two euler's angles are the same

    :param rpy_1: the first orientation in euler's angle
    :type rpy_1: list
    :param rpy_2: the second orientation in euler's angle
    :type rpy_2: list
    :param tolerance: the absolute tolerance for the comparison
    :type tolerance: float
    :return: True if the two orientations are deemed the same
    :rtype: bool
    """
    if len(rpy_1) != 3 or len(rpy_2) != 3:
        return False
    qx1, qy1, qz1, qw1 = tf.transformations.quaternion_from_euler(*rpy_1)
    qx2, qy2, qz2, qw2 = tf.transformations.quaternion_from_euler(*rpy_2)    
    cos_phi_half = math.fabs(qx2 * qx1 + qy2 * qy1 + qz2 * qz1 + qw2 * qw1) # angle between orientations
    return cos_phi_half >= math.cos(tolerance / 2.0)

def in_region(x:float, y:float, bbox_as_list:list) -> bool:
    """ Test if the point (x, y) is in the bbox
    
    :param x: the x value of the point
    :type x: float
    :param y: the y value of the point
    :type y: float
    :param bbox_as_list: the bounding square
    :type bbox_as_list: (x1, y1, x2, y2)
    :return: True if the point (x, y) is within the bbox
    :rtype: bool
    """
    return (bbox_as_list[0] <= x <= bbox_as_list[2]) and (bbox_as_list[1] <= y <= bbox_as_list[3])

# -----------------------------------------------
# functions for creating Pose and PoseStamp objects

def xyzrpy_to_pose_stamped(xyzrpy:list, reference_frame:str) -> PoseStamped:
    """ Convert a pose in the form of xyzrpy (list) to a PoseStamped object
    
    :param xyzrpy: a list of 6 numbers
    :type xyzrpy: list
    :param reference_frame: the reference frame of this pose
    :type reference_frame: str
    :return: the created Pose object according to the parameter 
    :rtype: Pose
    """
    return conversions.list_to_pose_stamped(xyzrpy, target_frame=reference_frame)

def pose_stamped_to_xyzrpy(the_pose:PoseStamped) -> list:
    """Convert a pose in a PoseStamped object to a (xyzrpy) list 

    :param the_pose: The pose
    :type the_pose: PoseStamped
    :return: The pose in xyzrpy
    :rtype: list
    """
    return conversions.pose_to_list(the_pose.pose)

def create_pose(xyzrpy:list) -> Pose:
    """ Convert a pose in the form of xyzrpy (list) to a Pose object
    
    :param xyzrpy: a list of 6 numbers
    :type xyzrpy: list
    :return: the created Pose object according to the parameter 
    :rtype: Pose
    """
    return conversions.list_to_pose(xyzrpy)

# -----------------------------------------------
# functions for creating Pose and PoseStamp objects
def create_pose_stamped(xyzrpy:list, target_frame:str) -> PoseStamped:
    """ Convert a pose in the form of xyzrpy (list) to a PoseStamped object
    
    :param xyzrpy: a list of 6 numbers
    :type xyzrpy: list
    :param reference_frame: the reference frame of this pose
    :type reference_frame: str
    :return: the created Pose object according to the parameter 
    :rtype: Pose
    """
    return xyzrpy_to_pose_stamped(xyzrpy, target_frame)

# -----------------------------------------------
# functions for creating Constraints objects

# - using the current pose as the reference
def create_path_orientation_constraint(link_name:str, current_pose:PoseStamped, x_axis:float, y_axis:float, z_axis:float) -> OrientationConstraint:
    """ Return a populated orientation constraint
    
    :param link_name: the name of the robot_link subject to the constraint
    :type link_name: str
    :param current_pose: The current pose of the link that is used as the reference
    :type current_pose: PoseStamped
    :param x_axis: The absolute tolerance in the x axis
    :type x_axis: float
    :param y_axis: The absolute tolerance in the y axis
    :type y_axis: float
    :param z_axis: The absolute tolerance in the z axis
    :type z_axis: float
    :return: The populated orientation constraint
    :rtype: OrientationConstraint
    """
    constraint = OrientationConstraint()
    constraint.header = current_pose.header
    constraint.link_name = link_name
    constraint.orientation = current_pose.pose.orientation
    constraint.absolute_x_axis_tolerance = x_axis
    constraint.absolute_y_axis_tolerance = y_axis
    constraint.absolute_z_axis_tolerance = z_axis
    constraint.weight = 1
    return constraint

def create_path_joint_constraint(joint_name:str, min_value:float, max_value:float) -> JointConstraint:
    """ Return a populated joint constraint
    
    :param joint_name: the name of the joint subject to the constraint
    :type joint_name: str
    :param min_value: the allowable minimum joint value
    :type min_value: float
    :param max_value: the allowable maximum joint value
    :type max_value: float
    :return: The populated joint constraint
    :rtype: JointConstraint
    """
    constraint = JointConstraint()
    constraint.joint_name = joint_name
    constraint.tolerance_above = max_value
    constraint.tolerance_below = 0
    constraint.position = min_value
    constraint.weight = 1
    return constraint

def create_position_constraint_on_link_orientation(link_name:str, current_pose:PoseStamped, dimensions_in_pose_orientation:list) -> PositionConstraint:
    """ return a populated PositionContraint  
    
    :param link_name: the name of the robot_link subject to the constraint
    :type link_name: str
    :param current_pose: the reference pose based on which the dimension in the position constraint is defined, with the reference pose at the centroid 
    :type current_pose: PoseStamped
    :param dimensions_in_pose_orientation: the dimension of the box that defines the position constraint
    :type dimensions_in_pose_orientation: list
    :return: the populated PositionContraint 
    :rtype: PositionConstraint
    """
    constraint = PositionConstraint()
    constraint.header.frame_id = current_pose.header.frame_id
    constraint.link_name = link_name
    constraint.target_point_offset = Vector3(0, 0, 0)
    s:SolidPrimitive = SolidPrimitive()
    s.dimensions = dimensions_in_pose_orientation
    s.type = s.BOX
    b = BoundingVolume()
    b.primitives.append(s)
    b.primitive_poses.append(current_pose.pose)
    constraint.constraint_region = b
    constraint.weight = 1
    return constraint

def create_position_constraint(link_name:str, reference_frame:str, xyz:list, dimensions:list) -> PositionConstraint:
    """ return a populated PositionContraint  
    
    :param link_name: the name of the robot_link subject to the constraint
    :type link_name: str
    :param reference_frame: the reference frame defining the orientation of the xyz and dimensions
    :type reference_frame: str
    :param xyz: the centroid of the box that defines the position constraint
    :type xyz: list    
    :param dimensions: the dimension of the box that defines the constraint
    :type dimensions: list
    :return: the populated PositionContraint 
    :rtype: PositionConstraint
    """    
    xyz.extend([0, 0, 0])
    the_pose:PoseStamped = conversions.list_to_pose_stamped(xyz, reference_frame)
    constraint = create_position_constraint_on_link_orientation(link_name, the_pose, dimensions)
    return constraint

def create_position_constraint_from_bbox(link_name:str, reference_frame:str, bbox:list) -> PositionConstraint:
    """ return a populated PositionContraint  
    
    :param link_name: the name of the robot_link subject to the constraint
    :type link_name: str
    :param reference_frame: the reference frame defining the orientation of the xyz and dimensions
    :type reference_frame: str
    :param bbox: the bounding box that defines the position constraint
    :type bbox: a list of 6 numbers (min_x, min_y, min_z, max_x, max_y, max_z)    
    :return: the populated PositionContraint 
    :rtype: PositionConstraint
    """   
    if not (bbox is not None and len(bbox) == 6):
        print(f'create_position_constraint_from_bbox: parameter (bbox) should be a list of 6 numbers -> fix the missing value at the function call')
        raise AssertionError(f'A parameter is invalid')
    dimensions = [bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2]]
    xyz = [(bbox[3] + bbox[0]) / 2, (bbox[4] + bbox[1]) / 2, (bbox[5] + bbox[2]) / 2]
    xyz.extend([0, 0, 0])
    the_pose:PoseStamped = conversions.list_to_pose_stamped(xyz, reference_frame)
    constraint = create_position_constraint_on_link_orientation(link_name, the_pose, dimensions)
    return constraint     

def create_code_to_string_map(the_class, keep_full_uppercase:bool=True, omit_begin_underscore:bool=True) -> dict:
    """ A utility for converting the uppercase variables of a class into a string map for conversion from variable values to variable names.
        It is applied on non-Enum class and provides a similar functionality of printing out the name of an Enum value
        
    :param the_class: The class that contains the variables
    :type the_class: A Python class definition
    :param keep_full_uppercase: Extracts only the variables in full uppercase, defaults to True
    :type keep_full_uppercase: bool, optional
    :param omit_begin_underscore: Omits the variables beginning with an underscore, defaults to True
    :type omit_begin_underscore: bool, optional
    :return: the string map (variable value, variable names)
    :rtype: dict
    """
    map = dict()
    keys = the_class.__dict__.keys()
    for key in keys:
        if omit_begin_underscore and key.startswith('_'):
            continue
        if keep_full_uppercase and key.isupper():
            value = the_class.__dict__[key]
            map[value] = key
    return map

# -----------------------------------------
# Convenient maps for GoalStatus and MoveItErrorCodes

MOVEIT_ERROR_CODE_MAP = create_code_to_string_map(MoveItErrorCodes) 
GOAL_STATUS_MAP = create_code_to_string_map(GoalStatus) 