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
class CommanderStates(Enum):
    READY = 0
    BUSY = 1
    SUCCEEDED = 2
    ABORTED = 3
    ERROR = 4

class TaskStates(Enum):
    INVALID = -1
    STANDBY = 0
    SUBMITTED = 1
    WORKING = 2
    SUCCEEDED = 3
    CANCELLED = 4
    ABORTED = 5
    FAILED = 6

COMPLETION_STATES = [
    TaskStates.SUCCEEDED, 
    TaskStates.ABORTED, 
    TaskStates.CANCELLED, 
    TaskStates.FAILED, 
    TaskStates.INVALID
] 

class ConstraintOptions(Enum):
    POSITION = 0
    ORIENTATION = 1
    JOINT = 2

if __name__ == '__main__':
    state = CommanderStates.BUSY
    print(state)