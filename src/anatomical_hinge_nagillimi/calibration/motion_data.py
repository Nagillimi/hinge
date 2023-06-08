from typing import List
from anatomical_hinge_nagillimi.kinematics import Kinematic
from anatomical_hinge_nagillimi.sensor_collection import SensorCollection

class MotionData:
    def __init__(self):
        self.sensorData: List[SensorCollection] = []
        
        # recognize data signals and assign kinematic status
        self.kinematic = Kinematic()


    def update(self, sensorData: SensorCollection):
        self.sensorData.append(sensorData)
        self.kinematic.update(sensorData)
