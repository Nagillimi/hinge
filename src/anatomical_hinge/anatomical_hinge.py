from constants import Constants
from state import State
from data import Data
from sensor_collection import SensorCollection
from joint import HingeJoint
from calibration.axis.axis import AxisCalibration
from calibration.pose.pose import PoseCalibration
from result.calibration_result import CalibrationResult
from result.temporal_result import TemporalResult
from result.result import Status
from kinematics import Kinematics
from utilities.temporal import DetectDownsampling

class AnatomicalHinge:
    def __init__(self) -> None:
        self.collection = SensorCollection()
        self.kinematics = Kinematics()
        self.detectDownsampling = DetectDownsampling()
        self.axisCalibration = AxisCalibration()
        self.poseCalibration = PoseCalibration()
        self.hingeJoint = HingeJoint()
        self.state = 1
        self.status = Status()
        self.mappedOperation: Status = {
            State.DETECT_DOWNSAMPLING_INDEX : self.detectDownsamplingIndex(),
            State.CALIBRATING               : self.calibrate(),
            State.RUN                       : self.run()
        }
    

    # Generic list type, easy API.
    # Converts to internal sensor collection used by the algorithms
    def update(self, data: Data) -> float:
        self.collection.update(data)
        self.status = self.mappedOperation[self.state]()

        # IF the dict map doesn't work...
        # if self.state == State.DETECT_DOWNSAMPLING_INDEX:
        #     self.status = self.detectDownsamplingIndex()
        # elif self.state == State.CALIBRATING:
        #     self.status = self.calibrate()
        # elif self.state == State.RUN:
        #     self.status = self.update()

        return self.hingeJoint.combinedAngle
    

    # Reports the current status string
    def getStatusString(self) -> str:
        return self.status.value


    # Call outside of calibration to detect the incoming avg data stream latency
    def detectDownsamplingIndex(self) -> Status:
        result = self.detectDownsampling.update(self.collection.a1.ts)
        print(result.value)
        if result == TemporalResult.OPTIMAL:
            Constants.ALGORITHM_DOWNSAMPLING_INDEX = self.detectDownsampling.index
            self.state += 1
        return result


    # run axis estimation
    # if return true, set the j vectors in the o vector class
    # run pose estimtation (one & done)
    # set the j & o vectors to the angle class
    def calibrate(self) -> Status:
        result = self.axisCalibration.calibrate(self.collection)
        print(result.value)
        if result == CalibrationResult.CALIBRATION_SUCCESS:
            # set the same motion data for pose calibration
            self.poseCalibration.setAxis(self.axisCalibration)
            self.poseCalibration.calibrate(self.axisCalibration.motionData)
            self.hingeJoint.setCalibration(self.axisCalibration, self.poseCalibration)
            self.state += 1
        return result


    # Normal run call for calculating the anatomical hinge joint
    def run(self) -> Status:
        result = self.hingeJoint.update()
        print(result.value)
        return result
