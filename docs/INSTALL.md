# Installation Guide for The Arm Commander Package
![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Installation (Docker)

The Dockerfile for building the environment is in the root directory of this repository. It has been tested in Ubuntu 20.04. Download the file to your local disk and change directory to where the file is located. 
```
docker build --tag arm_commander - < Dockerfile
```
The image building may take some time. When the build is completed, execute the following to check if the image `arm_commander` is there.
```
docker image ls
```
Execute below to allow RViz in the container to display a GUI on the host.
```
xhost +
```
Start a container based on the image.
```
docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" arm_commander
```
Inside the container, start the panda robot arm model in RViz
```
roslaunch panda_moveit_config demo.launch
```
Then execute the demo program.
```
cd ~/arm_commander_ws
source devel/setup.bash
rosrun arm_commander commander_demo.py
```



## Installation (Non-Docker)

#### Install Moveit 

The non-docker method assumes the starting point of having ROS Noetic installed. 

```
sudo apt-get update
sudo apt-get install ros-noetic-moveit -y
```

#### Install the Arm Commander

Create a catkin workspace.
```
mkdir -p ~/arm_commander_ws/src
cd ~/arm_commander_ws/src
```

Clone this repository to the `src` directory.
```
git clone git@github.com:REF-RAS/arm_commander.git
```

#### Install a Robot Arm Model

The demo program and the tutorial programs are designed to work with the dimension of a Panda.

```
cd ~/arm_commander_ws/src
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel 
```

#### Build the Packages

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

[![Watch the demo](https://img.youtube.com/vi/YleDRs649VA/0.jpg)](https://www.youtube.com/watch?v=YleDRs649VA)


## Links 

- [README: The Arm Commander Package](../README.md)


## Authors

Dr Andrew Lui, Senior Research Engineer <br />
Dr Dasun Gunasinghe, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Feb 2024
