import math
import random
from anatomical_hinge_nagillimi.utilities.spherical import SphericalCoordinate

class XSphere:
    def __init__(
            self,
            t1 = random.random(),
            p1 = random.random(),
            t2 = random.random(),
            p2 = random.random()
    ):
        self.vector1 = SphericalCoordinate(t1, p1)
        self.vector2 = SphericalCoordinate(t2, p2)


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
    