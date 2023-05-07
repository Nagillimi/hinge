from kinematics import KinematicState
from sensor_collection import SensorCollection

class MotionData:
    def __init__(self) -> None:
        self.sensorData = [SensorCollection()]
        self.kinematicState = [KinematicState()]


    def update(self, sensorData: SensorCollection, kinematicState: KinematicState):
        self.sensorData.append(sensorData)
        self.kinematicState.append(kinematicState)
