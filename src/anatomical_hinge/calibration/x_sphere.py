import math
from typing import Optional
import numpy as np
from utilities.sphere import SphericalCoordinate

class XSphere:
    def __get_imu1(self) -> SphericalCoordinate:
        return self.imu1
    
    # def _set_imu1(self, t1 = Optional[float], p1 = Optional[float]):
    #     self.imu1 = SphericalCoordinate(t1 or np.random(), p1 or np.random())
    
    def __get_imu2(self) -> SphericalCoordinate:
        return self.imu2
    
    # def _set_imu2(self, t2 = Optional[float], p2 = Optional[float]):
    #     self.imu2 = SphericalCoordinate(t2 or np.random(), p2 or np.random())

    imu1: SphericalCoordinate = property(__get_imu1) # , _set_imu1)
    imu2: SphericalCoordinate = property(__get_imu2) # , _set_imu2)

    def __init__(
            self,
            t1 = Optional[float],
            p1 = Optional[float],
            t2 = Optional[float],
            p2 = Optional[float]
    ):
        self.imu1 = SphericalCoordinate(t1 or np.random(), p1 or np.random())
        self.imu2 = SphericalCoordinate(t2 or np.random(), p2 or np.random())


    # Returns contents of non-iterable x as an iterable list
    def getAsList(self) -> list:
        return [
            self.imu1.theta,
            self.imu1.phi,
            self.imu2.theta,
            self.imu2.phi,
        ]
    

    # Performs sine operation on the non-iterable contents of x
    def sin(self) -> list:
        return [
            math.sin(self.imu1.theta),
            math.sin(self.imu1.phi),
            math.sin(self.imu2.theta),
            math.sin(self.imu2.phi),
        ]
    

    # Performs cosine operation on the non-iterable contents of x
    def cos(self) -> list:
        return [
            math.cos(self.imu1.theta),
            math.cos(self.imu1.phi),
            math.cos(self.imu2.theta),
            math.cos(self.imu2.phi),
        ]
    