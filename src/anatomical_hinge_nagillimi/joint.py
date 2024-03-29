import math
import numpy as np
from anatomical_hinge_nagillimi.calibration.motion_data import MotionData
from anatomical_hinge_nagillimi.calibration.solution_set import SolutionSet
from anatomical_hinge_nagillimi.utilities.historic import HistoricNumber
from anatomical_hinge_nagillimi.sensor_collection import SensorCollection
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.result.joint_result import HingeJointResult

class HingeJoint:
    def __init__(self):
        # Angles
        self.accelAngle = 0.0
        self.gyroIntegrand = HistoricNumber()
        self.gyroAngle = HistoricNumber()
        self.combinedAngle = HistoricNumber()

        # Initial condition variables
        self.areInitialConditionsSet = False
        self.tempBuffer = []

        # # JCS vectors
        self.x1 = np.zeros(shape=(1, 3))
        self.y1 = np.zeros(shape=(1, 3))
        self.x2 = np.zeros(shape=(1, 3))
        self.y2 = np.zeros(shape=(1, 3))

        # Motion data stored collecting data and stamping kinematics
        self.motionData = MotionData()


    # Set past calibration j & o vectors
    def setCalibration(self, axis: SolutionSet, pose: SolutionSet):
        self.j1 = axis.x.vector1.toRectangular()
        self.j2 = axis.x.vector2.toRectangular()
        self.o1 = pose.x.vector1.toRectangular()
        self.o2 = pose.x.vector2.toRectangular()


    # Set the joint coordinate system with c orthogonal to j1 & j2
    def setCoordinateSystem(self):
        self.x1 = np.cross(self.j1, Constants.C_VECTOR)
        self.y1 = np.cross(self.j1, self.x1)
        self.x2 = np.cross(self.j2, Constants.C_VECTOR)
        self.y2 = np.cross(self.j2, self.x2)


    # Update hinge joint angle based on current sensor data
    def update(self, collection: SensorCollection) -> HingeJointResult:
        self.motionData.update(collection)

        self.updateAccelBasedAngle()
        if not self.areInitialConditionsSet: return self.setInitialConditions()
        self.updateGyroBasedAngle()
        self.updateCombinedAngle()

        return HingeJointResult.STREAMING
            

    def updateAccelBasedAngle(self):
        sensorData = self.motionData.sensorData[-1]
        a1 = np.array(sensorData.a1.raw.current()) - (
            np.cross(sensorData.g1.raw.current(), np.cross(sensorData.g1.raw.current(), self.o1))
            + np.cross(sensorData.g1.deriv.current(), self.o1)
        )
        a1 = a1 / np.linalg.norm(a1)
        a1_2d = [np.dot(a1, self.x1), np.dot(a1, self.y1)]
        
        a2 = np.array(sensorData.a2.raw.current()) - (
            np.cross(sensorData.g2.raw.current(), np.cross(sensorData.g2.raw.current(), self.o2))
            + np.cross(sensorData.g2.deriv.current(), self.o2)
        )
        a2 = a2 / np.linalg.norm(a2)
        a2_2d = [np.dot(a2, self.x2), np.dot(a2, self.y2)]

        # https://www.mathworks.com/matlabcentral/answers/9330-changing-the-atan-function-so-that-it-ranges-from-0-to-2-pi#answer_12844
        self.accelAngle = math.atan2(np.cross(a1_2d, a2_2d), np.dot(a1_2d, a2_2d))

            
    def setInitialConditions(self) -> HingeJointResult:
        if Constants.USE_AVG_ACCEL_IC:
            self.tempBuffer.append(self.accelAngle)

            if len(self.tempBuffer) < Constants.NUM_SAMPLES_AVG_ACCEL_IC:
                return HingeJointResult.SETTING_INITIAL_CONDITIONS
            
            avgIC = np.average(self.tempBuffer)
            self.gyroAngle.shift(avgIC)
            self.combinedAngle.shift(avgIC)
        else:
            self.gyroAngle.shift(self.accelAngle)
            self.combinedAngle.shift(self.accelAngle)
        self.areInitialConditionsSet = True
        return HingeJointResult.INITIAL_CONDITIONS_SET


    def updateGyroBasedAngle(self):
        sensorData = self.motionData.sensorData[-1]
        self.gyroIntegrand.shift(
            np.dot(sensorData.g1.raw.current(), self.j1)
            - np.dot(sensorData.g2.raw.current(), self.j2)
        )
        self.gyroAngle.shift(
            self.gyroAngle.past
            + sensorData.a1.ts.delta() * self.gyroIntegrand.delta() / 2000
        )


    def updateCombinedAngle(self):
        self.combinedAngle.shift(
            Constants.SENSOR_FUSION_WEIGHT * self.accelAngle
            + (1 - Constants.SENSOR_FUSION_WEIGHT) * self.combinedAngle.past - self.gyroAngle.delta()
        )
