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