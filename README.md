# The Arm Commander Package
![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<!--
Replace REPO_USER, & REPO_NAME in the lines below to get more auto-generated badges
![Primary language](https://img.shields.io/github/languages/top/REPO_USER/REPO_NAME)
[![License](https://img.shields.io/github/license/REPO_USER/REPO_NAME)](./BSD.txt)
-->

The **Arm Commander** is a library providing an enhanced interface for robot arm manipulation. It is designed to wrap around a robot arm movement planner (i.e. robotic manipulation platforms) such as [Moveit](https://ros-planning.github.io/moveit_tutorials/) or [Armer](https://github.com/qcr/armer) and to offer features that achieve more with less programming:

- Multiple forms of movement target specification 
    - Pose
    - PoseStamped
    - A list of 6 numbers (xyzrpy) 
    - A list of 7 numbers (xyzqqqq)
    - The position components with defaults (x, y, and z)
        - The missing parameters are defaulted to the current values
    - The rotation components with defaults (roll, pitch, and yaw)
        - The missing parameters are defaulted to the current values
    - Displacement components (dx, dy, dz)
- Simple switch between cartesian path and planned path.
    - In a move command along a cartesian path, fractional execution is supported.
- Addition and removal of collision objects for path planning and pose transform 
    - Primitive shapes (box and sphere) and custom shapes are supported.
    - Pose transform between collision objects and robot links is supported through the tranform tree.
    - Attach and detach of objects for grab and drop simulation.
- Asynchronous and synchronous command support
    - State transition model for command management.
    - Convenient functions for waiting, polling, and aborting of commands.
- Addition and removal of contraints
    - Helper function for the creation of rotation, position, and joint-space constraints
- Agnostic to the robotic manipulation platform design-wise (though this implementation supports only Moveit)
- Agnostic to the robot arm model (through the underlying robotic manipulation platform)

## Installation

The arm_commander stems from an architecture that is robot model agnostic and arm manipulation platform agnostic. It hides away the
details and offers a consistent application programming interface. 

This implementation of arm_commander is specific to Moveit Version 1 (and therefore ROS Noetic). An environment installed with Moveit + ROS Noetic is needed. For your convenience, please refer to the docker image from the [docker deployment](https://github.com/REF-RAS/docker_deployment) repository of the REF-RAS group.

### Install Moveit (Non-Docker Method) 

The non-docker method assumes the starting point of having ROS Noetic installed. 

```
sudo apt-get update
sudo apt-get install ros-noetic-moveit -y
```

### Install Arm Commander

Create a catkin workspace.
```
mkdir -p ~/arm_commander_ws/src
cd ~/arm_commander_ws/src
```

Clone this repository to the `src` directory.
```
git clone git@github.com:REF-RAS/arm_commander.git
```

### Install a Robot Arm Model

The demo program and the tutorial programs are designed to work with the dimension of a Panda.

```
cd ~/arm_commander_ws/src
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel 
```

### Build the Packages

```
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Demonstration

The demo program is located at `examples/commander_demo.py`. To look at the demo, fire up the Panda model on RViz.
```
roslaunch panda_moveit_config demo.launch
```
Then execute the demo program.
```
cd ~/arm_commander_ws
source devel/setup.bash
/usr/bin/python3 ./src/arm_commander/arm_commander/commander_demo.py
```

Video of the demo program
[![Watch the demo](https://img.youtube.com/vi/YleDRs649VA/0.jpg)](https://www.youtube.com/watch?v=YleDRs649VA)

## Tutorials

A set of tutorials and example programs are provided in this package.

- [Tutorial: Programming with the Arm Commander Package (Part 1)](docs/TUTORIAL_PART1.md)
- [Tutorial: Programming with the Arm Commander Package (Part 2)](docs/TUTORIAL_PART2.md)


## API Documentation

API Documentation can be found in `/docs/build/html`. It will be available through Github Pages soon.

<!-- [API Documentation](https://REF-RAS.github.io/robotarchi/) is available for reference. -->

## Authors

Dr Andrew Lui, Senior Research Engineer <br />
Dr Dasun Gunasinghe, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Feb 2024
