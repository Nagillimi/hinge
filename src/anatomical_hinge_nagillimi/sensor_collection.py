from data import Data
from sensor import Sensor, SensorType

class SensorCollection:
    def __init__(self) -> None:
        self.a1 = Sensor(1, SensorType.Accelerometer)
        self.g1 = Sensor(2, SensorType.Gyroscope)
        self.a2 = Sensor(3, SensorType.Accelerometer)
        self.g2 = Sensor(4, SensorType.Gyroscope)

    def update(self, data: Data) -> None:
        self.a1.update(data.ts, data.a1)
        self.g1.update(data.ts, data.g1)
        self.a2.update(data.ts, data.a2)
        self.g2.update(data.ts, data.g2)
