from anatomical_hinge_nagillimi.data import Data
from anatomical_hinge_nagillimi.sensor import Sensor, SensorType

class SensorCollection:
    def __init__(self):
        self.a1 = Sensor(1, SensorType.Accelerometer)
        self.g1 = Sensor(2, SensorType.Gyroscope)
        self.a2 = Sensor(3, SensorType.Accelerometer)
        self.g2 = Sensor(4, SensorType.Gyroscope)


    def __str__(self) -> str:
        return "Collection: \n\ta1 = {}\n\tg1 = {}\n\ta2 = {}\n\tg2 = {}".format( 
            self.a1.raw.current(),
            self.g1.raw.current(),
            self.a2.raw.current(),
            self.g2.raw.current()
        )

    def update(self, data: Data):
        self.a1.update(data.ts, data.a1)
        self.g1.update(data.ts, data.g1)
        self.a2.update(data.ts, data.a2)
        self.g2.update(data.ts, data.g2)
