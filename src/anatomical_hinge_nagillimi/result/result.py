from typing import Union
from result.calibration_result import CalibrationResult
from result.joint_result import HingeJointResult
from result.kinematics_result import KinematicResult
from result.temporal_result import TemporalResult

# Union type for all results, to collect into a single status variable
Status = Union[CalibrationResult, HingeJointResult, KinematicResult, TemporalResult]
# Status = Enum('Status', [(i.name, i.value) for i in chain(
#     CalibrationResult, HingeJointResult, KinematicResult, TemporalResult)])
