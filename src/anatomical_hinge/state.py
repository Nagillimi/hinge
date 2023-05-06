from enum import Enum

class State(Enum):
    # analyzes incoming data to provide an appropriate downsampling index for the algorithms
    DETECT_DOWNSAMPLING_INDEX = 1
    # joint axis and pose estimations
    CALIBRATING = 2
    # hinge angle streaming, all algorithms are finished
    RUN = 3
    # used to initialize all states and restart
    RESET = 4
