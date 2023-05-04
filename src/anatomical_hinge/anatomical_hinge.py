from constants import Constants
from state import State
from data import Data
from sensor_collection import SensorCollection
from joint import HingeJoint
from calibration.axis.axis import AxisCalibration
from calibration.pose.pose import PoseCalibration
from result.calibration_result import CalibrationResult
from result.temporal_result import TemporalResult
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
        self.mappedOperation = {
            State.DETECT_DOWNSAMPLING_INDEX : self.detectDownsamplingIndex(),
            State.WAIT_FOR_MOTION           : self.kinematics.waitForMotion(self.collection),
            State.CALIBRATING               : self.calibrate(),
            State.WAIT_FOR_STILLNESS        : self.kinematics.waitForStillness(self.collection),
            State.RUN                       : self.run(),
            State.RESET                     : self.reset(),
        }
    

    # generic list type, easy API
    # converts to internal sensor collection used by the algorithms
    def update(self, data: Data) -> float:
        self.collection.update(data)
        self.mappedOperation[self.state]()
        return self.hingeJoint.combinedAngle


    def detectDownsamplingIndex(self):
        result = self.detectDownsampling.update(self.collection.a1.ts)
        print(result.value)
        if result == TemporalResult.OPTIMAL:
            Constants.ALGORITHM_DOWNSAMPLING_INDEX = self.detectDownsampling.index            


    # run axis estimation
    # if return true, set the j vectors in the o vector class
    # run pose estimtation (one & done)
    # set the j & o vectors to the angle class
    def calibrate(self):
        result = self.axisCalibration.calibrate(self.collection)
        print(result.value)
        if result == CalibrationResult.SUCCESS:
            # set the same motion data for pose calibration
            self.poseCalibration.setAxis(self.axisCalibration)
            self.poseCalibration.calibrate(self.axisCalibration.motionData)
            self.hingeJoint.setCalibration(self.axisCalibration, self.poseCalibration)
            self.state += 1


    def run(self):
        result = self.hingeJoint.update()
        print(result.value)
        if result

        # this.accelerometerBasedHingeAngle();
        # if(!this.initialConditionSet) {
        #     this.setInitialConditions();
        # } else { 
        #     this.gyroscopicBasedHingeAngle();
        #     this.combinedHingeAngle();
        # }
        pass


    def reset(self):
        pass
