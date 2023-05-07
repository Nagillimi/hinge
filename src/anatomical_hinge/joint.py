import numpy as np
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

        # vector for joint CS, where x1 is orthogonal to j1 and y1
        self.x1 = np.zeros(shape=(1, 3))

        # vector for joint CS, where y1 is orthogonal to j1 and x1
        self.y1 = np.zeros(shape=(1, 3))

        # vector for joint CS, where x2 is orthogonal to j2 and y2
        self.x2 = np.zeros(shape=(1, 3))

        # vector for joint CS, where y2 is orthogonal to j2 and x2
        self.y2 = np.zeros(shape=(1, 3))

        # Motion data stored for computing the pose calibration (requires the same data)
        self.sensorData = [SensorCollection()]


    # Set past calibration j & o vectors
    def setCalibration(self, axis: AxisCalibration, pose: PoseCalibration):
        self.j1 = axis.sols[-1].x.vector1.toRectangular()
        self.j2 = axis.sols[-1].x.vector2.toRectangular()
        self.o1 = pose.sols[-1].x.vector1.toRectangular()
        self.o2 = pose.sols[-1].x.vector2.toRectangular()


    # Update hinge joint angle based on current sensor data
    def update(self, collection: SensorCollection) -> HingeJointResult:
        self.sensorData.append(collection)
        