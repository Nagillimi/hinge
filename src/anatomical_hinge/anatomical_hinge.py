from anatomical_hinge.state import State
from anatomical_hinge.sensor import Sensor, SensorType
from anatomical_hinge.data import Data
from anatomical_hinge.calibration.axis import AxisCalibration
from anatomical_hinge.calibration.pose import PoseCalibration

class AnatomicalHinge:
    def __init__(self) -> None:
        self.operationMap = {
            State.INIT                      : self.initialize(),
            State.DETECT_DOWNSAMPLING_INDEX : self.detectDownsamplingIndex(),
            State.WAIT_FOR_MOTION           : self.waitForMotion(),
            State.JOINT_AXIS_INIT           : self.jointAxisInit(),
            State.JOINT_AXIS_ESTIMATION     : self.jointAxisEstimation(),
            State.JOINT_POSE_ESTIMATION     : self.jointPoseEstimation(),
            State.WAIT_FOR_STILLNESS        : self.waitingForStillness(),
            State.RUN                       : self.run(),
            State.RESET                     : self.reset(),
        }
        self.state = State.INIT
        self.stateMachine()
    
    def stateMachine(self) -> None:
        self.operationMap[self.state]()
        self.state = self.state(self.state.value + 1)

    def update(self, data: Data):
        self.a1.update(data.ts, data.a1)
        self.g1.update(data.ts, data.g1)
        self.a2.update(data.ts, data.a2)
        self.g2.update(data.ts, data.g2)
        self.stateMachine()

    def initialize(self):
        self.a1 = Sensor(1, SensorType.Accelerometer)
        self.g1 = Sensor(2, SensorType.Gyroscope)
        self.a2 = Sensor(3, SensorType.Accelerometer)
        self.g2 = Sensor(4, SensorType.Gyroscope)
        self.axisCalibration = AxisCalibration()
        self.poseCalibration = PoseCalibration()
        
    def detectDownsamplingIndex():
        print(1)