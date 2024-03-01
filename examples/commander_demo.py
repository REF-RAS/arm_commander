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

import sys, copy, threading, time, signal, math, traceback, os, yaml
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist

from arm_commander.moveit_tools import MOVEIT_ERROR_CODE_MAP, GOAL_STATUS_MAP
from arm_commander.states import ControllerState, GeneralCommanderStates
import arm_commander.moveit_tools as moveit_tools
from arm_commander.commander_moveit import GeneralCommander, GeneralCommanderFactory

class GeneralCommanderDemo():
    """ A demo program for the :class:'GeneralCommander', which illustrates the movement commands offered by 
    the GeneralCommander.  
    """
    def __init__(self, config_file='panda_demo.yaml'):
        """ The constructor

        :param config_file: a yaml file containing scene config for the deoo, defaults to 'panda_demo.yaml'
        :type config_file: str, optional
        """
        rospy.init_node('moveit_general_commander_node', anonymous=False)
        signal.signal(signal.SIGINT, self.stop)
        # rospy.on_shutdown(cb_shutdown)
        try:
            with open(os.path.join(os.path.dirname(__file__), f'config/{config_file}'), 'r') as f:
                self.demo_config = yaml.safe_load(f)
        except:
            rospy.logerr(f'Error in loading demo config file {config_file}')
            raise
        # TODO: config input to factory for group name and base
        self.arm_commander: GeneralCommander = GeneralCommanderFactory.get_object(self.demo_config['moveit_group_name'])
        # self.arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('xarm7', 'link_base')
        # self.arm_servo: CommanderServo = CommanderServo(moveit_group_name='xarm7', world_link='link_base')
        self.arm_commander.spin(spin_in_thread=True)
        
        self.test_main()
        
    def cb_shutdown(self):
        sys.exit(0)

    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.clear_path_constraints()
        self.arm_commander.reset_world()
        sys.exit(0)

    def _print_objects(self):
        objects = self.arm_commander.scene.get_objects()
        for object in objects:
            rospy.loginfo(f'Object: {self.arm_commander.scene.get_object_poses([object])}')

    def test_main(self):
        rospy.sleep(1.0)
        # self.test_0()
        self.test_1()
        self.test_2()
        self.test_3()
        # Servo test
        # self.test_4()

    def test_0(self):
        try:
            arm_commander = self.arm_commander
            arm_commander.info(print=True)
            arm_commander.current_joint_positons_as_list(print=True)                        
        except rospy.ROSInterruptException as e:
            rospy.logerr(e)

    def test_1(self):
        try:
            arm_commander = self.arm_commander
            arm_commander.current_joint_positions(print=True)
            arm_commander.current_joint_positons_as_list(print=True)
            arm_commander.info(print=True)
            arm_commander.set_max_cartesian_speed(0.1)
            arm_commander.set_planning_time(10.0)
            
            arm_commander.reset_world()
            arm_commander.set_workspace_walls(*(self.demo_config['scene']['workspace']))
            rospy.sleep(1.0)
                
            # Set named poses
            for named_pose in self.demo_config['scene']['named_poses'].keys():
                arm_commander.add_named_pose(named_pose, self.demo_config['scene']['named_poses'][named_pose])
            
            rospy.loginfo(f'=== Test 1: move commands and collision detection')
            arm_commander.wait_for_ready_to_move()
            rospy.loginfo(f'move_to_named_pose: stow')
            arm_commander.move_to_named_pose('stow', wait=True)
            arm_commander.reset_state()
            
            rospy.loginfo(f'move_to_named_pose: home')
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()
            
            arm_commander.add_box_to_scene('the_box', self.demo_config['scene']['the_box']['dimensions'], 
                                           self.demo_config['scene']['the_box']['position'],
                                           self.demo_config['scene']['the_box']['orientation'])
            rospy.sleep(1.0)
            
            # Move a displacement
            rospy.loginfo(f'move_displacement: dx')
            arm_commander.move_displacement(dx = 0.2, wait=True)
            arm_commander.reset_state() 
            rospy.loginfo(f'move_displacement: dy')       
            arm_commander.move_displacement(dy = 0.2, wait=True)
            arm_commander.reset_state()   
            rospy.loginfo(f'move_displacement: dz')    
            arm_commander.move_displacement(dz = 0.1, wait=True)
            arm_commander.reset_state()         

            # Move to a position using path planner
            rospy.loginfo(f'move_to_position: path mode, world frame x = 0.4')
            arm_commander.move_to_position(x = 0.4, wait=True)
            arm_commander.reset_state()           
            rospy.loginfo(f'move_to_position: path mode, world frame x = 0.2 y = -0.3')            
            arm_commander.move_to_position(x = 0.2, y = -0.3, wait=True)
            arm_commander.reset_state()       
            rospy.loginfo(f'move_to_position: path mode, world frame z = 0.8')
            arm_commander.move_to_position(z = 0.8, wait=True)
            arm_commander.reset_state()  
            
            # Move to a particular position in the world frame using cartesian movement
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()
            rospy.loginfo(f'move_to_position: cartesian mode, world frame x = 0.5')
            arm_commander.move_to_position(x = 0.5, cartesian=True, wait=True)
            arm_commander.reset_state()           
            rospy.loginfo(f'move_to_position: cartesian mode, world frame x = 0.3, y = 0.3')            
            arm_commander.move_to_position(x = 0.3, y = 0.3, cartesian=True, wait=True, accept_fraction=0.6)
            arm_commander.reset_state()       
            rospy.loginfo(f'move_to_position: cartesian mode, world frame x = 0.3, y = -0.4, z = 0.6')
            arm_commander.move_to_position(x = 0.3, y = -0.4, z = 0.6, cartesian=True, wait=True, accept_fraction=0.9)
            arm_commander.reset_state()
            rospy.loginfo(f'move_to_position: cartesian mode, world frame x = 0.6, y = 0.2, z = 0.3')
            arm_commander.move_to_position(x = 0.6, y = 0.2, z = 0.3, cartesian=True, wait=True)
            arm_commander.reset_state()    
        
            # Add a path orientation constraint which allows yaw rotation
            arm_commander.add_path_constraints(moveit_tools.create_path_orientation_constraint(
                self.arm_commander.END_EFFECTOR_LINK, arm_commander.pose_of_robot_link(), 0.05, 0.05, 6.28))

            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()
            rospy.loginfo(f'current pose: {arm_commander.pose_in_frame_as_xyzrpy(reference_frame="the_box")[:3]}')
            rospy.loginfo(f'move_to_position: path mode, box frame (0, 0, 0.35)')
            arm_commander.move_to_position(x = 0, y = 0, z = 0.35, reference_frame='the_box', wait=True)
            arm_commander.reset_state()       
            rospy.loginfo(f'current pose: {arm_commander.pose_in_frame_as_xyzrpy(reference_frame="the_box")[:3]}')            
            rospy.loginfo(f'move_to_position: path mode, box frame z = 0.65')            
            arm_commander.move_to_position(z = 0.65, reference_frame='the_box', wait=True)
            arm_commander.reset_state()       
            rospy.loginfo(f'current pose: {arm_commander.pose_in_frame_as_xyzrpy(reference_frame="the_box")[:3]}')
            rospy.loginfo(f'move_to_position: path mode, box frame x = 0.25, z = 0.31')
            arm_commander.move_to_position(x = 0.25, z = 0.31, reference_frame='the_box', wait=True)
            arm_commander.reset_state()         
            
            self.arm_commander.clear_path_constraints()   

            arm_commander.add_path_constraints(moveit_tools.create_path_orientation_constraint(
                self.arm_commander.END_EFFECTOR_LINK, arm_commander.pose_of_robot_link(),  0.05, 0.05, 6.28))

            # Rotate the end-effector
            rospy.loginfo(f'current orientation (the box): {arm_commander.pose_in_frame_as_xyzrpy(reference_frame="the_box")[3:]}')
            rospy.loginfo(f'current orientation: {arm_commander.pose_in_frame_as_xyzrpy()[3:]}') 
            rospy.loginfo(f'rotate (the_box): yaw = 3.14')
            arm_commander.rotate_to_orientation(yaw=-3.14, reference_frame='the_box')
            arm_commander.reset_state()  
            
            self.arm_commander.clear_path_constraints()   
            
            rospy.loginfo(f'rotate (the_box): roll = -2.8')
            arm_commander.rotate_to_orientation(roll=-2.8, reference_frame='the_box')  
            arm_commander.reset_state() 
            rospy.loginfo(f'rotate (world): yaw = -3.4')
            arm_commander.rotate_to_orientation(yaw=-3.14)  
            arm_commander.reset_state()  
            rospy.loginfo(f'rotate (the_box): roll = -3.14, yaw = 3.14')
            arm_commander.rotate_to_orientation(roll=-3.14, yaw=-3.14, reference_frame='the_box')                                    
            arm_commander.reset_state()  
 
            # Move to poses of various positions and orientations
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()
            
            rospy.loginfo(f'current pose: {arm_commander.pose_in_frame_as_xyzrpy()}')
            target_pose:PoseStamped = moveit_tools.create_pose_stamped([0.4, -0.3, 0.6, -3.14, 0, 1.57], arm_commander.WORLD_REFERENCE_LINK)
            rospy.loginfo(f'move to target pose: {target_pose.pose}')
            arm_commander.move_to_pose(target_pose)
            arm_commander.reset_state()
            
            target_pose:PoseStamped = moveit_tools.create_pose_stamped([0.65, 0.1, 0.5, -3.14, 0, 0], arm_commander.WORLD_REFERENCE_LINK)
            rospy.loginfo(f'move to target pose: {target_pose.pose}')
            arm_commander.move_to_pose(target_pose)
            arm_commander.reset_state()
            
            target_pose:PoseStamped = moveit_tools.create_pose_stamped([0.0, 0.35, 0.3, -3.14, 0, 1.57], arm_commander.WORLD_REFERENCE_LINK)
            rospy.loginfo(f'move to target pose: {target_pose.pose}')
            arm_commander.move_to_pose(target_pose)
            arm_commander.reset_state()     
            
            # Collision with self
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()
            rospy.loginfo(f'collision with self (will be aborted)')
            arm_commander.move_to_position(x = 0.0, y = 0.0, z = 0.0, wait=True)
            arm_commander.reset_state()

            # Collision with the wall
            rospy.loginfo(f'collision with the wall (will be aborted)')
            the_wall = self.demo_config['scene']['workspace']
            arm_commander.move_to_position(x = the_wall[3] + 0.1, y = 0.0, z = 0.4, cartesian=True, wait=True)
            arm_commander.reset_state()
            rospy.loginfo(f'collision with the wall (will be aborted)')
            arm_commander.move_to_position(x = 0.0, y = 0.0, z = the_wall[5] + 0.1, wait=True)
            arm_commander.reset_state()           

            # Collision with the box
            rospy.loginfo(f'collision with the box (will be aborted)')
            the_box = self.demo_config['scene']['the_box']['position']
            arm_commander.move_to_position(x = 0.0, y = 0, z = 0.0, reference_frame='the_box', wait=True)             
            arm_commander.reset_state() 
            
             # Collison avoidance of sphere
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()
            
            arm_commander.pose_in_frame_as_xyzrpy(print=True)
            rospy.loginfo(f'collision avoidance of a sphere')
            the_sphere = self.demo_config['scene']['the_sphere']
            arm_commander.add_sphere_to_scene('the_sphere', the_sphere['dimensions'], 
                                              the_sphere['position'])
            rospy.sleep(1.0)
            
            rospy.loginfo(f'move to the other side of the sphere')
            arm_commander.move_to_position(y = the_sphere['position'][1] + 0.15, wait=True)
            arm_commander.reset_state()
            
            rospy.loginfo(f'return to home pose')
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()    
            
            rospy.loginfo(f'move to the other side of the sphere by cartesian (will be aborted)')
            arm_commander.move_to_position(y = the_sphere['position'][1] + 0.15, cartesian=True, wait=True)
            arm_commander.reset_state() 
            
            rospy.loginfo(f'move to the other side of the sphere with orientation constraint')
            arm_commander.add_path_constraints(moveit_tools.create_path_orientation_constraint(
                self.arm_commander.END_EFFECTOR_LINK, arm_commander.pose_of_robot_link(), 0.05, 0.05, 0.05))
            arm_commander.move_to_position(y = the_sphere['position'][1] + 0.15, wait=True)
            arm_commander.reset_state()      
            rospy.loginfo(f'return to home pose')
            arm_commander.move_to_named_pose('home', wait=True)
                           
            arm_commander.clear_path_constraints()
            arm_commander.reset_state()                
            rospy.loginfo(f'=== Test 1: Finished')
                
        except rospy.ROSInterruptException as e:
            rospy.logerr(e)

    # test asynchronous commands
    def test_2(self):
        try:
            arm_commander = self.arm_commander
            # set workspace and named_poses
            arm_commander.reset_world()        
            arm_commander.set_workspace_walls(*(self.demo_config['scene']['workspace']))
            rospy.sleep(1.0)
            
            for named_pose in self.demo_config['scene']['named_poses'].keys():
                arm_commander.add_named_pose(named_pose, self.demo_config['scene']['named_poses'][named_pose])

            rospy.loginfo(f'=== Test 2: asynchronous commands')
            
            arm_commander.reset_state()
            arm_commander.move_to_named_pose('stow', wait=False)
            rospy.loginfo('move to stow command has been sent to the commander')
            while True:
                the_state = arm_commander.get_commander_state()
                if the_state not in [GeneralCommanderStates.BUSY]:
                    break
                rospy.sleep(0.1)
            rospy.loginfo(f'the commander has completed with the result: {the_state} {the_state.message}')
            # rospy.loginfo(f'Final result: {arm_commander.get_latest_moveit_feedback()}')
            arm_commander.reset_state() 
            rospy.loginfo(f'=== Test 2: Finished')
            
        except rospy.ROSInterruptException as e:
            rospy.logerr(e)   

   # test position constraint
    def test_3(self):
        try:
            arm_commander = self.arm_commander
            rospy.loginfo(f'=== Test 3: position constraints')
            # set workspace and named_poses
            arm_commander.reset_world()
            arm_commander.reset_state()
            arm_commander.set_workspace_walls(*(self.demo_config['scene']['workspace']))
            rospy.sleep(1.0)
            
            for named_pose in self.demo_config['scene']['named_poses'].keys():
                arm_commander.add_named_pose(named_pose, self.demo_config['scene']['named_poses'][named_pose])
            rospy.loginfo('return to stow')
            arm_commander.move_to_named_pose('stow', wait=True)
            arm_commander.reset_state()
            
            end_effector_link = arm_commander.END_EFFECTOR_LINK
            world_frame:str = arm_commander.WORLD_REFERENCE_LINK            
            current_pose: PoseStamped = arm_commander.pose_in_frame(end_effector_link)
            current_pose_xyzrpy: list = arm_commander.pose_in_frame_as_xyzrpy(end_effector_link)
            
            rospy.loginfo(f'current pose of the end-effector in xyzrpy: {arm_commander.pose_in_frame_as_xyzrpy()}')
            
            # - test position constraint with dimension specified in the world frame's orientation
            the_constraint = moveit_tools.create_position_constraint(end_effector_link, world_frame, current_pose_xyzrpy[:3], 
                                                                     [0.05, 0.45, 0.05])
            rospy.loginfo(f'added position constraint')
            arm_commander.add_path_constraints(the_constraint)
            rospy.loginfo(f'move to y + 0.1') 
            arm_commander.move_to_position(y=current_pose_xyzrpy[1] + 0.1, wait=True)   
            arm_commander.reset_state()

            rospy.loginfo(f'move to y - 0.2')            
            arm_commander.move_to_position(y=current_pose_xyzrpy[1] - 0.2, wait=True)   
            arm_commander.reset_state()
            
            # violate the position constraint
            rospy.loginfo(f'move to x + 0.5 (violated)') 
            arm_commander.move_to_position(x=current_pose_xyzrpy[0] + 0.5, wait=True)   
            arm_commander.reset_state()  
            
            # clear all the constraints
            rospy.loginfo(f'clear position constraint')
            arm_commander.clear_path_constraints()
            rospy.loginfo(f'move to x + 0.5')  
            arm_commander.move_to_position(x=current_pose_xyzrpy[0] + 0.5, wait=True)   
            arm_commander.reset_state()
            
            # - test constraint defined based on the link orientation (yaw is 1.57 or the end-effector is rotated a quarter along the y axis)
            rospy.loginfo(f'return to home') 
            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state()

            current_pose: PoseStamped = arm_commander.pose_in_frame(end_effector_link)
            current_pose_xyzrpy: list = arm_commander.pose_in_frame_as_xyzrpy(end_effector_link)
            rospy.loginfo(f'Current pose of the end-effector in xyzrpy: {current_pose_xyzrpy}')
            # NOTE: This test is dependent on the default orientation of the end-effector of the robot model used
            # panda_link8 end effector default orientation is z downward and x y at 45 degrees 
            # dimension[0] is outward-right (+x and -y) and dimension[1] is inward-right (-x and -y)
 
            the_constraint = moveit_tools.create_position_constraint_on_link_orientation(arm_commander.END_EFFECTOR_LINK, current_pose, 
                                            [1.0, 0.1, 0.1])
            rospy.loginfo(f'added position constraint defined in link orientation')
            arm_commander.add_path_constraints(the_constraint)

            # violate the position constraint
            rospy.loginfo(f'move to y + 0.1 (violated)') 
            arm_commander.move_to_position(y=current_pose_xyzrpy[1] + 0.1, wait=True)   
            arm_commander.reset_state()

            rospy.loginfo(f'move to y - 0.2 (violated)') 
            arm_commander.move_to_position(y=current_pose_xyzrpy[1] - 0.2, wait=True)   
            arm_commander.reset_state()
            
            # agree with the position constraint
            rospy.loginfo(f'move to x + 0.2 and y - 0.2') 
            arm_commander.move_to_position(x=current_pose_xyzrpy[0] + 0.2, y=current_pose_xyzrpy[1] - 0.2,  wait=True)   
            arm_commander.reset_state()  
            
            rospy.loginfo(f'clear position constraint')
            arm_commander.clear_path_constraints()           
            
            rospy.loginfo(f'return to stow') 
            arm_commander.move_to_named_pose('stow', wait=True)
            arm_commander.reset_state()  
            
            # - test constraint defined based on bbox
            rospy.loginfo(f'Current pose of the end-effector in xyzrpy: {arm_commander.pose_in_frame_as_xyzrpy()}')      
            
            the_constraint = moveit_tools.create_position_constraint_from_bbox(end_effector_link, world_frame, [0.2, -0.5, 0.2, 0.8, 0.5, 0.6])
            rospy.loginfo(f'added position constraint defined as bbox [0.2, -0.5, 0.2, 0.8, 0.5, 0.6]')
            arm_commander.add_path_constraints(the_constraint)
            
            # violate the position constraint
            rospy.loginfo(f'move to x = 0.1 (violated)') 
            arm_commander.move_to_position(x = 0.1, wait=True)   
            arm_commander.reset_state()           
            
            rospy.loginfo(f'move to y = 0.6 (violated)') 
            arm_commander.move_to_position(y = 0.6, wait=True)   
            arm_commander.reset_state()    
            
            rospy.loginfo(f'move to z = 0.7 (violated)') 
            arm_commander.move_to_position(z = 0.7, wait=True)   
            arm_commander.reset_state()          
            
            # agree with the position constraint      
            rospy.loginfo(f'move to x = 0.65') 
            arm_commander.move_to_position(x = 0.65, wait=True)   
            arm_commander.reset_state()           
            
            rospy.loginfo(f'move to y = -0.3') 
            arm_commander.move_to_position(y = -0.3, wait=True)   
            arm_commander.reset_state()    
            
            rospy.loginfo(f'move to z = 0.5') 
            arm_commander.move_to_position(z = 0.5, wait=True)   
            arm_commander.reset_state()      
            
            rospy.loginfo(f'clear position constraint')
            arm_commander.clear_path_constraints()
                         
            rospy.loginfo(f'return to stow') 
            arm_commander.move_to_named_pose('stow', wait=True)
            
            arm_commander.reset_state() 
            
            rospy.loginfo(f'=== Test 3: Finished')
                        
        except rospy.ROSInterruptException as e:
            rospy.logerr(e)     

    def test_4(self):
        """Servo Test
        NOTE: This needs the controller manager running (to switch correctly)
        NOTE: Moveit servo is slightly complicated in its dependencies required for functionality 
        """
        try:
            arm_commander = self.arm_commander
            arm_commander.reset_world()
            arm_commander.set_workspace_walls(-1.0, -1.0, 0, 1.0, 1.0, 1.0)
            arm_commander.move_to_named_pose('home', wait=False)
            rospy.loginfo(f'=== Test 4: servoing')
            # Attempt controller switch
            if arm_commander.controller_select(controller_type=ControllerState.SERVO):
                rospy.loginfo(f"Successfully Switched to SERVO Controller")
                twist = Twist()
                # Move forward (in x) for 5 seconds
                twist.linear.x = 0.1
                arm_commander.servo_robot(twist=twist, time=5)

                # Set back to normal Trajectory control
                if arm_commander.controller_select(controller_type = ControllerState.TRAJECTORY):
                    rospy.loginfo(f"Successfully Switched back to TRAJECTORY Controller")
                else:
                    rospy.logerr(f"Failed to Switch back to TRAJECTORY Controller")
            else:
                rospy.logwarn(f"Failed to Switch to SERVO Controller")

            arm_commander.move_to_named_pose('home', wait=True)
            arm_commander.reset_state() 
            
            rospy.loginfo(f'=== Test 4: Finished')
            
        except rospy.ROSInterruptException as e:
            rospy.logerr(e)   

if __name__ == '__main__':
    tester = GeneralCommanderDemo()