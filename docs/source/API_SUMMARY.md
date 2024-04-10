# A Summary of the API

![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

In addition to the functions for initialization and system settings, the application programming interface (API) can be divided into three groups of functions:
- Issuing and tracking move commands 
- Defining the workspace, collision objects, and custom frames of reference
- Querying the status of the robot arm and the system

This page provides a summary of the API. Refer to the [Full API Reference](arm_commander_modules.rst) for details.

## Initialization and System Settings

The **Arm Commander** is a Python object of the class `GeneralCommander` that represents the commander of a particular __arm__ or manipulation device. An application can use more than one `GeneralCommander` object if a robot arm platform comprising multiple manipulators.

### Factory for Creating Arm Commander

The function `get_object` of the class `GeneralCommanderFactory` returns the __singleton__ `GeneralCommander` object for a particular manipulator name. The current version is coupled with Moveit 1 and the moveit group name is to be provided to the factory. 

#### GeneralCommanderFactory

| Function | Parameters | Remarks |
| -------- | ---------- | ------- |
| get_object | The manipulator group name and optionally the name of the world reference frame | |

### System Parameters

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| info | Optionally print to the screen | Returns key information of the manipulator as a string |
| get_end_effector_link | | Returns the name of the end-effector |
| get_world_reference_frame | | Returns the name of the world reference frame |

### System States

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| get_commander_state |  | Returns the current state of the commander |
| spin | Execute in the current thread or a new thread | Starts the arm commander in a new thread by default or a blocking call |
| wait_for_ready_to_move | Optionally the timeout | Blocks until the manipulator is ready for move commands or timeout |
| wait_for_busy_end | | Blocks until the arm commander is not in BUSY state (move command completed) |
| reset_state | | Clear the status of the previous move command and ready for the next |

### System Parameters

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| set_max_cartesian_speed | The max speed | |
| set_planning_time | The max planning time | |
| set_goal_tolerance | A list of 3 numbers  | Joint-space, position, and orientation |
| get_goal_tolerance |  | |

### Named Poses in Joint-Space

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| add_named_pose | The name and joint-values | |
| forget_named_pose | The name | |

### Moveit Status

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| get_latest_moveit_feedback |  | Returns the latest `MoveGroupActionFeedback` received |
| get_error_code |  | Returns the latest error code received |


### Example: Essential Startup Sequence in Applications

The following shows the essential `GeneralCommander` startup sequence in applications.
```python
arm_commander: GeneralCommander = GeneralCommanderFactory.get_object('panda_arm')
arm_commander.spin(spin_in_thread=True)
arm_commander.wait_for_ready_to_move()
```

## Querying the status of the robot arm

The `GeneralCommander` is the access point for the services, such as querying the status of the robot arm.

### Joint-Space Status

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| current_joint_positions |  | Returns joint-positions as key-value pairs |
| current_joint_positons_as_list |  | Returns joint-positions as a list of numbers |

### Pose in the Arm Commander Frame 

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| pose_of_robot_link | The name of robot link | Returns PoseStamped of a robot link |
| rpy_of_robot_link | The name of robot link | Returns a list (rpy) of a robot link |
| xyzrpy_of_robot_link | The name of robot link | Returns a list (xyzrpy) of a robot link |

### Pose of Any Object in Any Frame of Reference

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| pose_in_frame | The link name and reference frame | Returns PoseStamped |
| pose_in_frame_as_xyzq | The link name and reference frame| Returns a list (xyzqqqq)|
| pose_in_frame_as_xyzrpy | The link name and reference frame| Returns a list (xyzrpy)|
| transform_pose | The current pose and the target frame | Returns PoseStamped in the target frame |

## Issuing Move Commands 

Many functions in the `GeneralCommander` allows the use of the current value of a position or orientation component as the default value.  

The `wait` parameter is another common feature that specifies the call is asynchronous or blocking.  

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| move_to_named_pose | The name |  |
| move_displacement | The dx, dy, and dz | cartesian path |
| move_to_position | The target x, y, and z, the reference frame, and path planning | |
| rotate_to_orientation | The target roll, pitch, and yaw, the reference frame, and path planning | |
| move_to_pose | Pose, PoseStamped, a 6-list (xyzrpy) or a 7-list (xyzqqqq) | |

The following functions accepts multiple positions or poses in one move command. The path is always cartesian.

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| move_to_multi_positions | A list of 3-tuple (x, y, z) or 3-list [x, y, z], and the reference frame | Supports default value in waypoints |
| move_to_multi-poses | A list of waypoints, each of which can be a Pose, PoseStamped, a 6-list (xyzrpy) or a 7-list (xyzqqqq) | Default value in waypoints not supported

### Example: Issuing an Asynchronous Move Command

The following shows an example of issuing an asynchronous move command
```python
arm_commander.move_to_position(x = -0.6, y = 0.2, wait=False)        
while True:
    the_state = arm_commander.get_commander_state()
    if the_state not in [GeneralCommanderStates.BUSY]:
        break
    rospy.sleep(0.1)
arm_commander.reset_state()
```
### Example: Issuing an Multi-Waypoints Move Command

The following shows an example of issuing a move command comprising multiple waypoints.
```python
xyzrpy_list = [(0.6, 0.0, 0.4, 3.14, 0, 0), 
            (0.6, 0.2, 0.5, 3.14, 0, 0), 
            (0.6, 0.2, 0.6, 3.14, 0, 1.58)]

arm_commander.move_to_multi_poses(waypoints_list=xyzrpy_list, wait=True)

the_state = arm_commander.get_commander_state()
if the_state == GeneralCommanderStates.SUCCEEDED:
    print('The multi move pose was successful')
elif the_state in [GeneralCommanderStates.ABORTED, GeneralCommanderStates.ERROR]:
    print(f'Error: {arm_commander.get_error_code()}')
```

## Defining the workspace, collision objects, and custom frames of reference

### General Scene Management

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| reset_world | | Clear the workspace and all collision objects in the world |
| set_workspace_walls | A 3D bounding-box | |

### Objects in the Scene

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| add_object_to_scene | The name, object mesh file, pose, and reference frame |  |
| add_sphere_to_scene | The name, dimension, pose, and reference frame | |
| add_box_to_scene | The name, dimension, pose, and reference frame | |
| list_object_names | | Returns a list of object names |
| remove_object | The object name | |
| attach_object_to_end_effector | The object name | |
| detach_all_from_end_effector | The object name | |

The object mesh file accepts absolute file path or package relative file path.

#### Example: Adding a collision object

The following shows an example of issuing an asynchronous move command
```python
arm_commander.add_object_to_scene('the_teapot', '/path_to_file/utah_teapot.stl', [0.5, 1.2, 0.3], [0, 0, 3.14])
```

### Custom Transform in the Scene

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| add_custom_transform | The name, xyz, rpy and the parent_frame of the custom transform | | 
| update_custom_transform_pose | The updated xyz, rpy or the parent frame of the named custom transform | the xyz, rpy, and the parent frame are optional|
| remove_custom_transform| The name of the custom transform | |

### Path Constraints

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| add_path_constraints | The constraint |  |
| clear_path_constraints | | Remove all constraints from the arm commander |
| disable_path_constraints | | Disable the current constraints for the next move commands |
| enable_path_constraints | | Enable the current constraints for the next move commands |

## The Moveit Tools

The `moveit_tools` module offers helper functions for creating constraint objects and handling poses.

### Path Constraint Objects

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| create_path_joint_constraint | | Constraint joints' values |
| create_path_orientation_constraint | | Constraint the orientation of the end-effector |
| create_position_constraint_on_link_orientation | | Constraint the position in the reference frame of the end-effector |
| create_position_constraint | | Constraint the position of the end-effector in the world frame |
| create_position_constraint_from_bbox | Constraint the position of the end-effector in the world frame using a 3D bounding box|

## The Pose Tools

### Pose Conversion

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| list_to_pose | A list (xyzrpy or xyzqqqq) | Returns Pose  |
| list_to_pose_stamped | A list (xyzrpy or xyzqqqq) and the target frame | Returns PoseStamped  |
| pose_to_xyzq | The pose | Returns a list (xyzqqqq) |
| pose_to_xyzrpy | The pose | Returns a list (xyzrpy) |
| list_to_xyzq | A list (xyzrpy or xyzqqqq) | Returns a list (xyzqqqq) |
| list_to_xyzrpy | A list (xyzrpy or xyzqqqq) | Returns a list (xyzrpy) |

### Pose Comparison

| Functions | Parameters | Remarks |
| -------- | ---------- | ------- |
| same_pose_with_tolerence | | Compare two poses |
| same_joint_values_with_tolerence | | Compare two poses in the joint-space |
| same_rpy_with_tolerence | | Compare two rotation |
| same_xyz_with_tolerence | | Compare two position |
| in_region | | Returns true if (x,y) is within a 2D bounding box|

### Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Feb 2024
