#!/usr/bin/env python3
# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

__author__ = 'Andrew Lui'
__author__ = 'Dasun Gunasinghe'
__copyright__ = 'Copyright 2024, Research Engineering Facility (REF)'
__license__ = 'BSD-3'
__version__ = '0.1.0'
__email__ = 'robotics.ref@qut.edu.au'
__status__ = 'Development'

from enum import Enum

# --- Definition of ENUMs ---
class RobotAgentStates(Enum):
    READY = 0
    BUSY = 1
    SUCCEEDED = 2
    ABORTED = 3

class MoveitActionStates(Enum):
    IDLE = 0
    PLANNING = 1
    MONITOR = 2
    ERROR = 3
    COMPLETED = 4