from sensor_collection import SensorCollection
from constants import Constants
from result.joint_result import HingeJointResult
from calibration.axis.axis import AxisCalibration
from calibration.pose.pose import PoseCalibration


class HingeJoint:
    def __init__(self):
        self.accelBasedAngle = 0.0
        self.gyroBasedAngle = 0.0
        self.combinedAngle = 0.0

        # Motion data stored for computing the pose calibration (requires the same data)
        self.motionData = [SensorCollection()]


    def setCalibration(self, axis: AxisCalibration, pose: PoseCalibration):
        pass

    def update(self, collection: SensorCollection) -> HingeJointResult:
        self.motionData.append(collection)
        