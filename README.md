# The Arm Commander Package
![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

The following package presents a convenient interface to:
- The Moveit2! Move Group ROS Node
- The Moveit2! Moveitpy Python binding of the moveit_cpp API

The fundamental approach in Moveit2! is explained by reviewing their [concept diagram](https://moveit.ros.org/documentation/concepts/). The main interface by the user (or program) is via three points:
- ***A Command Interface:*** In Moveit2! this is typically through the C++ move group interface API called *move_group_interface*, which then makes ROS calls (actions, topics, services) to the Move Group Node.
- ***RVIZ plugins:*** These are typical calls through the RVIZ launch when brining up a robot config
- ***Direct calls to the Move Group Node:*** This is typically through the available ROS topics, actions, and services provided by the Move Group node. 

However, a noted difference between Moveit! and Moveit2! is that the python equivalent of the moveit inteface (also known as ***moveit_commander***) is not available, with a priority to focus on migration of the C++ varient (also known as the ***move_group_interface***). This presents some challenges with respect to python-based development, typically using ***py_trees*** for high-level behaviour design. There are, of course, alternatives in C++ to py_trees that would make a pure C++ development pipeline feasible (such as [BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP))

For the time being this branch presents a snap-shot of a short investigation into the advantages, shortcomings and possible pathways to use Moveit2 and ROS2 in future project for the REF-RAS team.

## Package Structure and Current Snapshot of Template
The package is currently structured in the following way:
- __arm_commander__: the main package
    - __arm_commander_cpp__: 
        - a rclcpp package (barebones) setup for future development (should the existing *move_group_interface* be a desirable usecase)
        - We could essentially make an equivalent structure to the existing ROS1 commander (main branch) but in C++ to gain the same features. Noting that py_trees would require an additional wrapper in the middle or adoption of the behaviour architecture/design to Behaviour.CPP (which would be additional effort either way)
    - __arm_commander_py__: a rclpy package that contains python files to interface to Moveit2
        - ***commander_move_group.py***: 
            - This is a General Commander package that interfaces to Moveit! directly through the *Move Group ROS Node* and circumvents the need to communicate to the *move_group_interface*. 
            - This is the approach that as adopted by [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) and presents flexibility in addapting it towards the desired structure of *move_group_interface* with more of a design narrative matching the inputs/outputs the REF/RAS team desire with respect to the manipulation design architecture using py_trees.
            - Note that, while this method is functional, certain features hidden within *move_group_interface*, such as named pose capture, are functions that would need re-implementation (so there is scope for more work and maintenance on our behalf)
            - The overall benefit of effort would be a module that encompasses the *move_group_interface* AND the *general commander* rather than two separate modules (assuming that the *Move Group Node* functionality is to remain more or less the same without too much deviation over time)
        - ***commander_moveitpy.py***: 
            - This is another version of a General Commander package that interfaces to Moveit! through a very new, direct C++ called *moveit_cpp* (utilising newly implemented python bindings in *moveit_py*). 
            - The desire here by PicNik is to have a direct and faster interface to Moveit! core features (implemented normally through the *Move Group Node*) without having to interact in and out of ROS. 
            - While the benefits are certainly desirable (i.e., faster calls, more real-time safe), functionality is currently still missing (in comparison to the existing pipeline, and the existing functionality is not directly one-to-one, meaning some calls and features are named differently or differently implemented). 
            - This being said, it is a new and active component of work that is improveing Moveit2 and would be the natural way forward should functionality appear for core features we need.

