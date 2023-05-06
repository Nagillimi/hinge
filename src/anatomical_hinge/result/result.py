from result.calibration_result import CalibrationResult
from result.joint_result import HingeJointResult
from result.temporal_result import TemporalResult

# Union type for all results, to collect into a single status variable
class Status(CalibrationResult, HingeJointResult, TemporalResult):
    pass