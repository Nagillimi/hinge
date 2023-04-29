from numpy import array
from utilities.historic import HistoricNumber
from anatomical_hinge.data import Data
from anatomical_hinge.sensor import Sensor, SensorType

class SensorCollection:
    a1 = Sensor(1, SensorType.Accelerometer)
    g1 = Sensor(2, SensorType.Gyroscope)
    a2 = Sensor(3, SensorType.Accelerometer)
    g2 = Sensor(4, SensorType.Gyroscope)

    def update(self, data: Data) -> None:
        self.a1.update(data.ts, data.a1)
        self.g1.update(data.ts, data.g1)
        self.a2.update(data.ts, data.a2)
        self.g2.update(data.ts, data.g2)


