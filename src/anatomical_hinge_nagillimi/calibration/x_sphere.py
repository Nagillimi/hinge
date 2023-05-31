import math
from typing import Optional
import numpy as np
from utilities.spherical import SphericalCoordinate

class XSphere:
    def __get_vector1(self) -> SphericalCoordinate:
        return self.vector1
    
    # def _set_imu1(self, t1 = Optional[float], p1 = Optional[float]):
    #     self.imu1 = SphericalCoordinate(t1 or np.random(), p1 or np.random())
    
    def __get_vector2(self) -> SphericalCoordinate:
        return self.vector2
    
    # def _set_imu2(self, t2 = Optional[float], p2 = Optional[float]):
    #     self.imu2 = SphericalCoordinate(t2 or np.random(), p2 or np.random())

    vector1: SphericalCoordinate = property(__get_vector1) # , _set_imu1)
    vector2: SphericalCoordinate = property(__get_vector2) # , _set_imu2)

    def __init__(
            self,
            t1 = Optional[float],
            p1 = Optional[float],
            t2 = Optional[float],
            p2 = Optional[float]
    ):
        self.vector1 = SphericalCoordinate(t1 or np.random(), p1 or np.random())
        self.vector2 = SphericalCoordinate(t2 or np.random(), p2 or np.random())


    # Returns contents of non-iterable x as an iterable list
    def getAsList(self) -> list:
        return [
            self.vector1.theta,
            self.vector1.phi,
            self.vector2.theta,
            self.vector2.phi,
        ]
    

    # Performs sine operation on the non-iterable contents of x
    def sin(self) -> list:
        return [
            math.sin(self.vector1.theta),
            math.sin(self.vector1.phi),
            math.sin(self.vector2.theta),
            math.sin(self.vector2.phi),
        ]
    

    # Performs cosine operation on the non-iterable contents of x
    def cos(self) -> list:
        return [
            math.cos(self.vector1.theta),
            math.cos(self.vector1.phi),
            math.cos(self.vector2.theta),
            math.cos(self.vector2.phi),
        ]
    