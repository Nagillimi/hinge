from state import State
from data import Data
from sensor_collection import SensorCollection
from calibration.axis import AxisCalibration
from calibration.pose import PoseCalibration
from error.calibration_error import CalibrationError
from anatomical_hinge.kinematics import Kinematics

class AnatomicalHinge:
    collection = SensorCollection()
    kinematics = Kinematics()
    axisCalibration = AxisCalibration()
    poseCalibration = PoseCalibration()
    hingeJoint = HingeJoint()

    def __init__(self) -> None:
        self.mappedOperation = {
            State.DETECT_DOWNSAMPLING_INDEX : self.detectDownsamplingIndex(self.collection),
            State.WAIT_FOR_MOTION           : self.kinematics.waitForMotion(self.collection),
            State.CALIBRATING               : self.calibrate(),
            State.WAIT_FOR_STILLNESS        : self.kinematics.waitForStillness(self.collection),
            State.RUN                       : self.hingeJoint.compute(self.collection),
            State.RESET                     : self.reset(),
        }
        self.state = 1
        self.stateMachine()
    
    # generic list type, easy API
    # converts to internal sensor collection used by the algorithms
    def update(self, data: Data) -> float:
        self.collection.update(data)
        self.mappedOperation[self.state]()
        return hingeJoint.angle

    def detectDownsamplingIndex():
        pass

    # run axis estimation
    # if return true, set the j vectors in the o vector class
    # run pose estimtation (one & done)
    # set the j & o vectors to the angle class
    def calibrate(self):
        result = self.axisCalibration.calibrate(self.collection)
        if result == CalibrationError.SUCCESS:
            # set the same motion data for pose calibration
            self.poseCalibration.setAxis(self.axisCalibration)
            self.poseCalibration.calibrate(self.axisCalibration.motionData)
            self.hingeJoint.setCalibration(self.axisCalibration, self.poseCalibration)
            self.state += 1
        else:
            print(result.value)

    def reset():
        pass
