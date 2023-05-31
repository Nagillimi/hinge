from constants import Constants
from data import Data
from details import Details
from state import State
from sensor_collection import SensorCollection
from joint import HingeJoint
from calibration.axis.axis import AxisCalibration
from calibration.pose.pose import PoseCalibration
from result.calibration_result import CalibrationResult
from result.temporal_result import TemporalResult
from result.result import Status
from kinematics import Kinematic
from utilities.temporal import DetectDownsampling

class AnatomicalHinge:
    def __init__(self) -> None:
        self.collection = SensorCollection()
        self.kinematics = Kinematic()
        self.detectDownsampling = DetectDownsampling()
        self.axisCalibration = AxisCalibration()
        self.poseCalibration = PoseCalibration()
        self.hingeJoint = HingeJoint()
        self.status = Status
        self.state = State()
        # self.mappedOperation: Status = {
        #     State.DETECT_DOWNSAMPLING_INDEX : self.detectDownsamplingIndex(),
        #     State.CALIBRATING               : self.calibrate(),
        #     State.RUN                       : self.run()
        # }


    # Generic list type, easy API.
    # Converts to internal sensor collection used by the algorithms
    def update(self, data: Data) -> float:
        self.collection.update(data)
        # self.status = self.mappedOperation[self.state]()

        # IF the dict map doesn't work...
        if self.state == State.DETECT_DOWNSAMPLING_INDEX:
            self.status = self.detectDownsamplingIndex()
        elif self.state == State.CALIBRATING:
            self.status = self.calibrate()
        elif self.state == State.RUN:
            self.status = self.run()

        return self.hingeJoint.combinedAngle
    

    # Reports the current status string (enumeration)
    def getStatusString(self) -> str:
        return self.status.value
    

    # Reports the current details of the algorithm as a JSON string
    def getDetailsJSON(self) -> str:
        return Details().createReportJSON(self)


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
        result = self.axisCalibration.update(self.collection)
        print(result.value)
        if result == CalibrationResult.SUCCESS:
            # set the same motion data for pose calibration
            self.poseCalibration.setAxis(self.axisCalibration)
            self.poseCalibration.update(self.axisCalibration.motionData)
            self.hingeJoint.setCalibration(self.axisCalibration, self.poseCalibration)
            self.hingeJoint.setCoordinateSystem()
            self.state += 1
        return result


    # Normal run call for calculating the anatomical hinge joint
    def run(self) -> Status:
        result = self.hingeJoint.update(self.collection)
        print(result.value)
        return result
