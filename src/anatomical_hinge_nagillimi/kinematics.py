from enum import Enum
from typing import List
import numpy as np
from anatomical_hinge_nagillimi.sensor_collection import SensorCollection
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
    MAX_BUFFER_LENGTH = 100

    ACCEL_INSTANT_STATE_THRESHOLD = 0.001
    GYRO_INSTANT_STATE_THRESHOLD = 0.005
    GYRO_DOT_INSTANT_STATE_THRESHOLD = 0.00001

    ACCEL_MOTION_THRESHOLD = 0.05
    GYRO_MOTION_THRESHOLD = 0.75
    CONSECUTIVE_MOTION_TIME_MS = 100

    ACCEL_STILLNESS_THRESHOLD = 0.05
    GYRO_STILLNESS_THRESHOLD = 0.25
    CONSECUTIVE_STILLNESS_TIME_MS = 200


    def __init__(self) -> None:
        self.buffer: List[StateBuffer] = []


    def update(self, collection: SensorCollection) -> None:
        A1 = np.linalg.norm(collection.a1.raw.current()) - 1.0 # removed gravity
        A2 = np.linalg.norm(collection.a2.raw.current()) - 1.0 # removed gravity
        G1 = np.linalg.norm(collection.g1.raw.current())
        G2 = np.linalg.norm(collection.g2.raw.current())
        G1DOT = np.linalg.norm(collection.g1.deriv.current())
        G2DOT = np.linalg.norm(collection.g2.deriv.current())

        # test motion index & push
        self.buffer.append(StateBuffer(
            self.getInstantState(A1, A2, G1, G2, G1DOT, G2DOT),
            int(collection.a1.ts.current)
        ))

        # slice the buffer to make circular
        if len(self.buffer) > Kinematic.MAX_BUFFER_LENGTH:
            self.buffer.pop(0)


    # Test instantaeneous state, called internally
    def getInstantState(self, A1, A2, G1, G2, G1DOT, G2DOT) -> KinematicState:
        if (A1 > Kinematic.ACCEL_INSTANT_STATE_THRESHOLD and A2 > Kinematic.ACCEL_INSTANT_STATE_THRESHOLD
            and G1 > Kinematic.GYRO_INSTANT_STATE_THRESHOLD and G2 > Kinematic.GYRO_INSTANT_STATE_THRESHOLD
            and G1DOT > Kinematic.GYRO_DOT_INSTANT_STATE_THRESHOLD and G2DOT > Kinematic.GYRO_DOT_INSTANT_STATE_THRESHOLD):
            return KinematicState.MOTION
        return KinematicState.STILL


    # Test if consecutive over required time amount
    def testForConsecutive(self) -> KinematicResult:
        if self.buffer[-1].state == KinematicState.MOTION:
            for bufferEl in reversed(self.buffer):
                if bufferEl.state != KinematicState.MOTION: break
                if self.buffer[-1].ts - bufferEl.ts >= Kinematic.CONSECUTIVE_MOTION_TIME_MS:
                    return KinematicResult.CONSECUTIVE_MOTION_DETECTED
            return KinematicResult.MOTION_DETECTED
        
        if self.buffer[-1].state == KinematicState.STILL:
            for bufferEl in reversed(self.buffer):
                if bufferEl.state != KinematicState.STILL: break
                if self.buffer[-1].ts - bufferEl.ts >= Kinematic.CONSECUTIVE_STILLNESS_TIME_MS:
                    return KinematicResult.CONSECUTIVE_STILLNESS_DETECTED
            return KinematicResult.STILLNESS_DETECTED
        
        return KinematicResult.MOTION_UNDER_THRESHOLD
