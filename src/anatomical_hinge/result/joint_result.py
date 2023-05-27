from enum import Enum

class HingeJointResult(Enum):
    SETTING_INITIAL_CONDITIONS = "Setting ICs from average buffer"
    INITIAL_CONDITIONS_SET = "ICs from average buffer are set"
    