import numpy as np

class Sensor:
    def __init__(self, type: str) -> None:
        # init 2d array
        self.raw = np.array([],[])
        self.type = type

    def update(self, newRaw) -> None:
        # append new data to beginning of 2d array
        self.raw = newRaw

    # tsArr : time array, where the first el is the latest ts (larger number)
    def calculateDerivative(self, tsArr: np.ndarray[np.uint32]) -> None:
        den = 2 * (tsArr[0] - tsArr[2])
        num = self.raw.current - self.raw.past2
        self.deriv = num / den

