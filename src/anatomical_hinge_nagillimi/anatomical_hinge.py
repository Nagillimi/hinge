from typing import Union
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.data import Data
from anatomical_hinge_nagillimi.details import Details
from anatomical_hinge_nagillimi.state import State
from anatomical_hinge_nagillimi.sensor_collection import SensorCollection
from anatomical_hinge_nagillimi.joint import HingeJoint
from anatomical_hinge_nagillimi.calibration.axis.axis import AxisCalibration
from anatomical_hinge_nagillimi.calibration.pose.pose import PoseCalibration
from anatomical_hinge_nagillimi.result.calibration_result import CalibrationResult
from anatomical_hinge_nagillimi.result.joint_result import HingeJointResult
from anatomical_hinge_nagillimi.result.kinematics_result import KinematicResult
from anatomical_hinge_nagillimi.result.temporal_result import TemporalResult
from anatomical_hinge_nagillimi.kinematics import Kinematic
from anatomical_hinge_nagillimi.utilities.temporal import DetectDownsampling

class AnatomicalHinge:
    def __init__(self) -> None:
        self.collection = SensorCollection()
        self.kinematics = Kinematic()
        self.detectDownsampling = DetectDownsampling()
        self.axisCalibration = AxisCalibration()
        self.poseCalibration = PoseCalibration()
        self.hingeJoint = HingeJoint()
        self.status = Union[CalibrationResult, HingeJointResult, KinematicResult, TemporalResult]
        self.state = State(1)
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

        return self.hingeJoint.combinedAngle.current
    

    # Reports the current status string (enumeration)
    def getStatusString(self) -> str:
        return str(self.status.value)
    

    # Reports the current details of the algorithm as a JSON string
    def getDetailsJSON(self) -> str:
        return Details().createReportJSON(self)


    # Call outside of calibration to detect the incoming avg data stream latency
    def detectDownsamplingIndex(self) -> TemporalResult:
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
    def calibrate(self) -> Union[CalibrationResult, KinematicResult]:
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
    def run(self) -> Union[HingeJointResult, KinematicResult]:
        result = self.hingeJoint.update(self.collection)
        print(result.value)
        return result
