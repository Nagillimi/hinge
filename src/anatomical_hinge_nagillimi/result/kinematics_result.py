from enum import Enum

class KinematicResult(Enum):
    MOTION_DETECTED = "Significant motion detected"
    CONSECUTIVE_MOTION_DETECTED = "Consecutive significant motion detected"
    MOTION_UNDER_THRESHOLD = "Insignificant motion detected"
    STILLNESS_DETECTED = "Stillness detected"
    CONSECUTIVE_STILLNESS_DETECTED = "Stillness detected"
