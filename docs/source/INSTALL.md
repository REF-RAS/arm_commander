# Installation Guide
![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

The docker based installation should be the easier than the non-docker counterpart, especially if you already have docker desktop already installed on your computer.

## Software Requirements
- [Moveit 1](https://ros-planning.github.io/moveit_tutorials/) 
- [ROS 1 Noetic](http://wiki.ros.org/noetic)
- Python 3.8 or above

## Installation (Docker)

The [Dockerfile](../../Dockerfile) for building the environment is in the root directory of this repository. It has been tested with Docker 25.0.3 and Ubuntu 20.04. Download the file to your local disk and change directory to where the file is located. 
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

### Install Moveit 

The non-docker method assumes the starting point of having ROS Noetic installed. 

```
sudo apt-get update
sudo apt-get install ros-noetic-moveit -y
```

### Install the Arm Commander

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
## The Demonstration Program

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

## Tutorial Programs

The tutorial programs are stored under the `examples` folder. They are divided into 6 folder.
```bash
├── examples
|  ├── collision    examples on collision avoidance and collision objects
|  ├── framenove    examples on custom frames and movememnt with reference to custom frames
|  ├── config       the config files for running the commander_demo.py on different robot arm models
|  ├── move         examples on basic movement
|  ├── multimove    examples on multi-waypoint movement
|  ├── named_poses  examples on movement to named poses and the loading of named poses specification
```

Latest update: Feb 2024
