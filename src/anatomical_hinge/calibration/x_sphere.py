import numpy as np

class XSphere:
    theta1: int
    phi1: int
    theta2: int
    phi2: int

    def __init__(self) -> None:
        self.set(0, 0, 0 , 0)

    def randomize(self) -> None:
        self.set(np.random(), np.random(), np.random(), np.random())

    def set(
            self,
            theta1: int,
            phi1: int,
            theta2: int,
            phi2: int) -> None:
        self.theta1 = theta1
        self.phi1 = phi1
        self.theta2 = theta2
        self.phi2 = phi2
        