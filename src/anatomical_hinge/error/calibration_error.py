from enum import Enum

class CalibrationResult(Enum):
    SUCCESS = "Calibration succeeded"
    NOT_ENOUGH_UNIQUE_MOTION = "Not enough unique motion data"
    MOTION_ERROR_ABOVE_THRESHOLD = "Sum of squares error of motion above threshold"
    