from anatomical_hinge.state import State
from anatomical_hinge.sensor import Sensor

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
        # if self.state == State.DETECT_DOWNSAMPLING_INDEX:
        #     self.detectDownsamplingIndex()
        # elif self.state == State.WAIT_FOR_MOTION:
        #     self.waitForMotion()
        # elif self.state == State.JOINT_AXIS_INIT:
        #     self.jointAxisInit()
        # elif self.state == State.JOINT_AXIS_ESTIMATION:
        #     self.jointAxisEstimation()
        # elif self.state == State.JOINT_POSE_ESTIMATION:
        #     self.jointPoseEstimation()
        # elif self.state == State.WAIT_FOR_STILLNESS:
        #     self.waitingForStillness()
        # elif self.state == State.RUN:
        #     self.run()
        # elif self.state == State.RESET:
        #     self.reset()

        self.operationMap[self.state]()
        self.state._generate_next_value_()

    def initialize(self):
        self.a1 = Sensor('accel')
        self.a2 = Sensor('accel')
        self.g1 = Sensor('gyro')
        self.g2 = Sensor('gyro')
        

    def detectDownsamplingIndex():
        print(1)