from itertools import chain
from enum import Enum
from result.calibration_result import CalibrationResult
from result.joint_result import HingeJointResult
from result.temporal_result import TemporalResult

# Union type for all results, to collect into a single status variable
Status= Enum('Status', [(i.name, i.value) for i in chain(CalibrationResult, HingeJointResult, TemporalResult)])
