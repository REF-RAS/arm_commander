# Programming Tutorial Part 1

Part 1 of the tutorial covers the basics of programming with the arm commander package, including the first programs covering the essentials of the package and the primitive move commands.

Follow the [Installation Guide](INSTALL.md) to get the arm commmander ready on your computer.

## Structure of the Example Program Folder

Example programs are provided for illustrating how to use the **general arm commander** package in programming robot arm manipulations. They are can found under the directory `examples`. The programs are divided into sub-directories:
```bash
├── examples
|  ├── collision    examples on collision avoidance, collision objects and path constraints
|  ├── framenove    examples on custom frames and movememnt with reference to custom frames
|  ├── config       the config files for running the commander_demo.py on different robot arm models
|  ├── move         examples on basic movement
|  ├── multimove    examples on multi-waypoint movement
|  ├── named_poses  examples on movement to named poses and the loading of named poses specification
```

## Programming Essentials

The essential Python modules are found in the following three files.
- `commander_moveit.py`: defines main classes including the `GeneralCommander`.
- `states.py`: defines the key `Enum` classes for state management.
- `moveit_tools.py`: defines utility and helper functions.

### Import the Arm Commander in Python

To use the Python arm_commander interface, import the modules in the concerned Python program file as below.

```python
from arm_commander.commander_moveit import GeneralCommander
from arm_commander.states import ControllerState, GeneralCommanderStates
import arm_commander.moveit_tools as moveit_tools
```

### The First Program

The program file `move/simple_move_1.py` illustrates the essentials of programming with the arm commander.

#### Create the General Arm Commander

The string parameter `panda_arm` specifies the __move_group__ as defined in the robot arm configuration (`panda_moveit_config`).

```python
arm_commander:GeneralCommander = GeneralCommander('panda_arm')
arm_commander.spin(spin_in_thread=True)
arm_commander.wait_for_ready_to_move()
```

#### Important: The Threading Model

The GeneralCommander object must run in a different thread from the client that uses the object. The independent thread can be created as part of the call to the `spin()` function as shown below. 

```python
arm_commander.spin(spin_in_thread=True)
```

Alternatively, the client can create a new thread and use it to execute the spin function. In this case, the parameter `spin_in_thread` should be False and the function call will __not__ return, as shown below.

```python
class ArmCommanderMoveExample():
    def __init__(self):
        ...
        arm_commander:GeneralCommander = GeneralCommander('panda_arm')
        self.arm_commander = arm_commander
        self.the_thread = threading.Thread(target=self.spin_commander, daemon=True)
        self.the_thread.start()
        ...

    def spin_commander(self):
        self.arm_commander.spin(spin_in_thread=False)
```

#### Graceful Handling of Program Interrupts

As a good practice, client programs of the general arm commander should abort any move command before termination (whether self-induced or otherwise). The following is recommended.

```python
class ArmCommanderMoveExample():
    def __init__(self):
        ...
        signal.signal(signal.SIGINT, self.stop)
        arm_commander: GeneralCommander = GeneralCommander('panda_arm')
        self.arm_commander = arm_commander
        ...

    def stop(self, *args, **kwargs):
        self.arm_commander.abort_move()
        self.arm_commander.clear_path_constraints() 
        self.arm_commander.reset_world()
```

#### Move to a XYZ Position

Make a call to one of the move functions of the arm commander to move the robot arm's end effector to a position. For example, the following call to `move_to_position` moves the end effector to the location (x, y, z) = (0.0, 0.0, 0.4).
```python
# send a move command
arm_commander.move_to_position(x = 0.6, y = 0.0, z = 0.4, wait=True)
arm_commander.reset_state()
```
The general arm commander uses states to regulate the interaction with the client program. For example, the state notifies the client if the 
command was successful or failed.
```python
...
arm_commander.move_to_position(x = 0.6, y = 0.0, z = 0.4, wait=True)
the_state = arm_commander.get_commander_state()
if the_state == GeneralCommanderStates.SUCCEEDED:
    print('The move was successful')
elif the_state in [GeneralCommanderStates.ABORTED, GeneralCommanderStates.ERROR]:
    print(f'Error: {arm_commander.get_error_code()}')
```

#### Reset to the Stow Pose

The robot arm may get stuck in the examples if its initial pose is problematic for the planned movement. The program file `reset_robot.py` can be used to reset the robot arm back to the stow pose, which should be a reliable pose for starting any movement.

#### The States of the General Arm Commander

The following figure shows the states of the general arm commander and their significance to the client programs

![General Commander States](../assets/GeneralCommanderStates.png)

### Asynchronous Commands

Specifying move commands asynchronously enables the execution of other tasks while the move command is being handled by the general arm commander. Set the parameter `wait` to False to signify that the command is asynchronous. The execution returns to the caller before the command is completed.

Addition logic is required for asynchronous move commands, such as a polling loop as below. The source code is from `simple_move_2.py`.

```python
...
arm_commander.move_to_position(x = -0.6, y = 0.2, wait=False)        
while True:
    the_state = arm_commander.get_commander_state()
    if the_state not in [GeneralCommanderStates.BUSY]:
        break
    time.sleep(0.1)
arm_commander.reset_state()
...
```
Alternatively, use the function `wait_for_busy_end()` for a blocking wait of the end of the `BUSY` state.

### Abort the Current Move Commands

The function `abort_move()` terminates the current move command. If the parameter `wait` is True, the function does not return until the abort has been
handled by the underlying platform completely and the state will become `GeneralCommanderStates.ABORTED`.

The following source code is from `simple_move_3.py`.
```python
...
arm_commander.move_to_position(x = -0.6, y = 0.2, wait=False)
# abort the command after 3 seconds and wait for the abort to take effect
time.sleep(3.0)
arm_commander.abort_move(wait=True)
arm_commander.reset_state()
...
```

## Move Commands to a Position or Orientation

### Move to a position

The function `move_to_position()` commands (the end-effector of) the arm to move to a position specified in XYZ. 
- The XYZ are specified as three component parameters.
- Their parameter values are defaulted to the current XYZ. In the second function call, the target of the move command is 
x (0.4), y (0.2) and the z will remain unchanged.
- The parameter `wait` specifies whether the move command is synchronous (True) or asynchronous (False). 
- A synchronous command call is blocking. The execution returns only after the command execution is completed.
- The function `reset_state()` must be called before issuing another move command to the general arm commander.

```python
# send a move command
arm_commander.move_to_position(x = 0.6, y = 0.0, z = 0.4, wait=True)
arm_commander.reset_state()
# send a move command
arm_commander.move_to_position(x = 0.4, y = 0.2, wait=True)
arm_commander.reset_state()
```


### Move using a displacement

The function `move_displacement()` commands the robot arm to move based on a displacement from the current position. The following source code is from `simple_move_4.py`.  The end-effector will move 10 cm ten times.
```python
for step in range(10):
    arm_commander.move_displacement(dy = 0.1, wait=True)
    arm_commander.reset_state()
```
### Rotate

The function `rotate_to_orientation()` commands the end-effector to rotate according to the given Euler's angles (roll, pitch, and yaw). The following source code comes from `simple_move_4.py`. 
```python
arm_commander.rotate_to_orientation(roll = 3.14, pitch = 0.0, yaw = 0.2, wait=True)
arm_commander.reset_state()
```
The default values of the three component parameters are the current values.

### Move to Both Position and Orientation

The function `move_pose()` commands the end-effector to move to a target pose, which can be of type `Pose` or `PoseStamped`. 
```python
pose = Pose()
pose.position.x = 0.4
pose.position.y = 0.2
pose.position.z = 0.6
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
pose.orientation.w = 1.0
arm_commander.move_to_pose(pose, wait=True)
arm_commander.reset_state()
```
The helper function `list_to_pose()` from the module `pose_tools` offers conversion of a list of `xyzrpy` or `xyzqqqq` into `Pose` or `PoseStamped`.
The following source code is from `frame_move_1.py`.

```python
xyzrpy = [0.4, 0.0, 0.4, 3.14, 0.0, 0.6]
arm_commander.move_to_pose(pose_tools.list_to_pose(xyzrpy), wait=True)
arm_commander.reset_state() 
```
The function `move_to_pose` can accept different pose formats, including list of `xyzrpy` or `xyzqqqq` and Pose and PoseStamped. The following passes the list of `xyzrpy` to the function without
the conversion.
```python
xyzrpy = [0.4, 0.0, 0.4, 3.14, 0.0, 0.6]
arm_commander.move_to_pose(xyzrpy, wait=True)
arm_commander.reset_state() 
```

![Animation of the Movement](../assets/ArmCommander-SimpleMove5.gif)

### Cartesian Movement

The function `move_position()` supports both cartesian path planning (with collision avoidance) and algorithmic path planning (based on a path planner). 
Setting the parameter `cartesian` to True turns on cartesian path planning.  The following source code, found in`simple_move_5.py` compares the two
types of path planning.

```python
# send a move command, z is defaulted to the current z value
logger.info(f'Go back to start')
arm_commander.move_to_position(x = 0.0, y = -0.5, z = 0.2, wait=True)            
arm_commander.reset_state()   

# send a move command moveing back to the original position, constrained cartesian movement
logger.info(f'From start to target (cartesian is True)')
arm_commander.move_to_position(x = 0.5, y = 0.0, z = 0.4, cartesian=True, wait=True)
arm_commander.reset_state()  
```

### Move to Multi-Positions or Multi-Poses Commands

The function `move_to_multi_poses()` commands the end-effector to move through several waypoint poses in a cartesian path. The waypoint poses are specified in a list, with each element can be a Pose, PoseStamped, 6-list (xyzrpy) or 7-list (xyzqqqq). The following source code comes from `multi_move_2.py`, which defines a list of 6 waypoints and commands the arm to move through these waypoints in one movement.
```python
xyzrpy_list = [(0.6, 0.0, 0.4, 3.14, 0, 0), 
            (0.6, 0.2, 0.5, 3.14, 0, 0), 
            (0.6, 0.2, 0.6, 3.14, 0, 1.58), 
            (0.6, 0.0, 0.7, 3.14, 0, 3.14), 
            (0.6, -0.2, 0.6, 3.14, 0, 1.58), 
            (0.6, -0.2, 0.5, 3.14, 0, 0), 
            (0.6, 0.0, 0.4, 3.14, 0, 0)]
# send a multi move command
arm_commander.move_to_multi_poses(waypoints_list=xyzrpy_list, wait=True)
```

Similarly the function `move_to_multi_positions()` also commands the end-effector to move through several positions while keeping the rotation same. The xyz positions are specified as a list of 3-list [x, y, z] or 3-tuple (x, y, z). This function has an additional feature of using the current position (at the start) as the default value when any of the 3-list or 3-tuple contains a None value. The following source code comes from `multi_move_1.py`, which defines a list of 6 xyz positions. Note that the x component of all positions is None, and the missing value will assume the current x position.

```python
xyz_list = [(None, 0.0, 0.4), (None, 0.2, 0.5), (None, 0.2, 0.6), (None, 0.0, 0.7), 
            (None, -0.2, 0.6), (None, -0.2, 0.5), (None, 0.0, 0.4)]
arm_commander.move_to_multi_positions(xyz_list=xyz_list, wait=True)
```

### References

- [Programming Tutorial Part 2)](TUTORIAL_PART2.md)

### Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Feb 2024
