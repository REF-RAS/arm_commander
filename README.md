# The Arm Commander Package
![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

The following package presents a convenient interface to:
- The Moveit2! Move Group ROS Node
- The Moveit2! Moveitpy Python binding of the moveit_cpp API

The fundamental approach in Moveit2! is explained by reviewing their [concept diagram](https://moveit.picknik.ai/main/doc/concepts/concepts.html). The main interface by the user (or program) is via three points:
- ***A Command Interface:*** In Moveit2! this is typically through the C++ move group interface API, which then makes ROS calls (actions, topics, services) to the Move Group Node.
- ***RVIZ plugins:*** These are typical calls through the RVIZ launch when brining up a robot config
- ***Direct calls to the Move Group Node:*** This is typically through the available ROS topics, actions, and services provided by the Move Group node. 

However, a noted difference between Moveit! and Moveit2! is that the python equivalent of the moveit commander is not available, with a priority to conduct implementations in 