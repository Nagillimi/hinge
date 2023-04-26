from enum import Enum

class State(Enum):
    # initializing class variables
    INIT = 1
    # analyzes incoming data to provide an appropriate downsampling index for the algorithms
    DETECT_DOWNSAMPLING_INDEX = 2
    # waiting for significant motion to continue
    WAIT_FOR_MOTION = 3
    # single state, initializes the joint axis algorithm
    JOINT_AXIS_INIT = 4
    # joint axis algorithm
    JOINT_AXIS_ESTIMATION = 5
    # joint position algorithm derived from the same motion data as the joint axis algorithm
    JOINT_POSE_ESTIMATION = 6
    # waiting for adequate stillness to set an appropriate IC for the integral-derived signals
    WAIT_FOR_STILLNESS = 7
    # hinge angle streaming, all algorithms are finished
    RUN = 8
    # used to initialize all states and restart
    RESET = 9
