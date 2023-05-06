from enum import Enum

class CalibrationResult(Enum):
    SUCCESS = "Calibration succeeded"
    WAITING_FOR_MOTION = "Waiting for motion"
    NOT_ENOUGH_UNIQUE_MOTION = "Not enough unique motion data yet"
    MOTION_ERROR_ABOVE_THRESHOLD = "Sum of squares error of motion above threshold"
    