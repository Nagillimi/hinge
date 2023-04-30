import numpy as np
from utilities.sphere import SphericalCoordinate

class XSphere:
    imu1 = SphericalCoordinate(np.random(), np.random())
    imu2 = SphericalCoordinate(np.random(), np.random())

    def set(
            self,
            theta1: float,
            phi1: float,
            theta2: float,
            phi2: float) -> None:
        self.imu1.theta = theta1
        self.imu1.phi = phi1
        self.imu2.theta = theta2
        self.imu2.phi = phi2
        