from enum import Enum
from anatomical_hinge_nagillimi.utilities.historic import HistoricPoint, HistoricNumber

class SensorType(Enum):
    Accelerometer = 'Accelerometer'
    Gyroscope = 'Gyroscope'

class Sensor:
    def __init__(self, id: int, type: SensorType):
        self.id = id
        self.type = type
        self.raw = HistoricPoint()
        self.deriv = HistoricPoint()
        self.ts = HistoricNumber()

    def update(self, newTs: int, newRaw: list):
        self.ts.shift(newTs)
        self.raw.shift(newRaw)
        if self.type == SensorType.Gyroscope:
            self.calculateDerivative()

    # tsArr : time array, where the first el is the latest ts (larger number)
    def calculateDerivative(self):
        den = 2 * (self.ts.current - self.ts.past2)
        newDeriv = [
            (self.raw.x.current - self.raw.x.past2) / den,
            (self.raw.y.current - self.raw.y.past2) / den,
            (self.raw.z.current - self.raw.z.past2) / den]
        self.deriv.shift(newDeriv) 
