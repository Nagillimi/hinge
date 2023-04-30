import numpy as np
import math

class SphericalCoordinate:
    theta = 0.0
    phi = 0.0

    def __init__(self, theta, phi) -> None:
        self.theta = theta
        self.phi = phi

    # Assumes a radius of 1 in spherical coordinates, converts spherical coords
    # into rectangular coords. Convection follows implementation from the 2020
    # article from Thomas Seel: https://www.mdpi.com/1424-8220/20/12/3534
    def toRectangular(self) -> np.array:
        return np.array([
            math.cos(self.theta) * math.cos(self.phi),
            math.cos(self.phi) * math.sin(self.theta),
            math.sin(self.phi)
        ])
