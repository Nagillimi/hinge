import numpy as np

class DataBuffer:
    ts: np.array(dtype=int)
    gyro1: np.array(dtype=float)
    gyro1Dot: np.array(dtype=float)
    accel1: np.array(dtype=float)
    gyro2: np.array(dtype=float)
    gyro2Dot: np.array(dtype=float)
    accel2: np.array(dtype=float)

    def __init__(self) -> None:
        self.set(
            ts=0,
            gyro1=0,
            gyro1Dot=0,
            accel1=0,
            gyro2=0,
            gyro2Dot=0,
            accel2=0
        )

    def set(self,
            ts: int,
            gyro1: float,            
            gyro1Dot: float,            
            accel1: float,            
            gyro2: float,            
            gyro2Dot: float,            
            accel2: float) -> None:
        self.ts = ts
        self.gyro1 = gyro1
        self.gyro1Dot = gyro1Dot
        self.accel1 = accel1
        self.gyro2 = gyro2
        self.gyro2Dot = gyro2Dot
        self.accel2 = accel2


