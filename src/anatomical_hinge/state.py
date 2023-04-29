from enum import Enum

class State(Enum):
    # analyzes incoming data to provide an appropriate downsampling index for the algorithms
    DETECT_DOWNSAMPLING_INDEX = 1
    # waiting for significant motion to continue
    WAIT_FOR_MOTION = 2
    # joint axis and pose estimations
    CALIBRATING = 3
    # waiting for adequate stillness to set an appropriate IC for the integral-derived signals
    WAIT_FOR_STILLNESS = 4
    # hinge angle streaming, all algorithms are finished
    RUN = 5
    # used to initialize all states and restart
    RESET = 6
