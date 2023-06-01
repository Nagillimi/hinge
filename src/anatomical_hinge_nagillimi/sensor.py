from enum import Enum
from anatomical_hinge_nagillimi.utilities.historic import HistoricPoint, HistoricNumber

class SensorType(Enum):
    Accelerometer = 'Accelerometer'
    Gyroscope = 'Gyroscope'

class Sensor:
    def __init__(self, id: int, type: SensorType) -> None:
        self.id = id
        self.type = type
        self.raw = HistoricPoint()
        self.deriv = HistoricPoint()
        self.ts = HistoricNumber()

    def update(self, newTs: int, newRaw: list) -> None:
        self.ts.shift(newTs)
        self.raw.shift(newRaw)
        if self.type == SensorType.Gyroscope:
            self.calculateDerivative()

    # tsArr : time array, where the first el is the latest ts (larger number)
    def calculateDerivative(self) -> None:
        den = 2 * (self.ts.current - self.ts.past2)
        self.deriv.x = (self.raw.x.current - self.raw.x.past2) / den
        self.deriv.y = (self.raw.y.current - self.raw.y.past2) / den
        self.deriv.z = (self.raw.z.current - self.raw.z.past2) / den
