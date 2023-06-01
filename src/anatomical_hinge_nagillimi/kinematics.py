from enum import Enum
import numpy as np
from anatomical_hinge_nagillimi.sensor_collection import SensorCollection
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.result.kinematics_result import KinematicResult

class KinematicState(Enum):
    MOTION = 1
    STILL = 2
    INSIGNIFICANT_MOTION = 3

class StateBuffer:
    def __init__(self, state: KinematicState, ts: int) -> None:
        self.state = state
        self.ts = ts

class Kinematic:
    ACCEL_MOTION_THRESHOLD = 0.5
    GYRO_MOTION_THRESHOLD = 0.75
    CONSECUTIVE_MOTION_TIME_MS = 20

    ACCEL_STILLNESS_THRESHOLD = 0.5
    GYRO_STILLNESS_THRESHOLD = 0.25
    CONSECUTIVE_STILLNESS_TIME_MS = 250


    def __init__(self) -> None:
        # self.motionBuffer = [(KinematicState, int)]
        self.buffer = [StateBuffer]


    def update(self, collection: SensorCollection) -> None:
        A1 = np.linalg.norm(collection.a1.raw.current) - Constants.GRAVITY
        A2 = np.linalg.norm(collection.a2.raw.current) - Constants.GRAVITY
        G1 = np.linalg.norm(collection.g1.raw.current)
        G2 = np.linalg.norm(collection.g2.raw.current)

        # test motion index & push
        self.buffer.append(StateBuffer(
            self.getInstantState(A1, A2, G1, G2),
            collection.a1.ts
        ))


    # Test instantaeneous state, called internally
    def getInstantState(self, A1, A2, G1, G2) -> KinematicState:
        if (
            A1 > Kinematic.ACCEL_MOTION_THRESHOLD and A2 > Kinematic.ACCEL_MOTION_THRESHOLD
            and G1 > Kinematic.GYRO_MOTION_THRESHOLD and G2 > Kinematic.GYRO_MOTION_THRESHOLD
        ):
            return KinematicState.MOTION
        elif (
            A1 < Kinematic.ACCEL_STILLNESS_THRESHOLD and A2 < Kinematic.ACCEL_STILLNESS_THRESHOLD
            and G1 < Kinematic.GYRO_STILLNESS_THRESHOLD and G2 < Kinematic.GYRO_STILLNESS_THRESHOLD
        ):
            return KinematicState.STILL
        else:
            return KinematicState.INSIGNIFICANT_MOTION


    # Test if consecutive over required time amount
    def testForConsecutive(self) -> KinematicResult:
        if self.buffer[-1].ts == KinematicState.MOTION:
            if self.buffer[-1][-1] - self.buffer[0][-1] >= Kinematic.CONSECUTIVE_MOTION_TIME_MS:
                self.buffer.pop(0)
                return KinematicResult.CONSECUTIVE_MOTION_DETECTED
            return KinematicResult.MOTION_DETECTED
        
        if self.buffer[-1][0] == KinematicState.STILL:
            if self.buffer[-1][-1] - self.buffer[0][-1] >= Kinematic.CONSECUTIVE_STILLNESS_TIME_MS:
                self.buffer.pop(0)
                return KinematicResult.CONSECUTIVE_STILLNESS_DETECTED
            return KinematicResult.STILLNESS_DETECTED
        
        return KinematicResult.MOTION_UNDER_THRESHOLD
