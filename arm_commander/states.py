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

from enum import Enum

class GeneralCommanderStates(Enum):
    INIT = -1
    READY = 0
    BUSY = 1
    SUCCEEDED = 2
    ABORTED = 3
    ERROR = 4

class ControllerState(Enum):
    TRAJECTORY = 0
    SERVO = 1
