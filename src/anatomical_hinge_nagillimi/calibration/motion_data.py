from kinematics import Kinematic
from sensor_collection import SensorCollection

class MotionData:
    def __init__(self) -> None:
        self.sensorData = [SensorCollection()]
        
        # recognize data signals and assign kinematic status
        self.kinematic = Kinematic()


    def update(self, sensorData: SensorCollection):
        self.sensorData.append(sensorData)
        self.kinematic.update(sensorData)