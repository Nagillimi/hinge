from numpy import array
from calibration.x_sphere import XSphere
from calibration.state_vector import AxisStateVector

# calculates the j vectors
class AxisCalibration:
    pSV = AxisStateVector()

    def __init__(self) -> None:
        self.x = XSphere()

    def calibrate(self) -> bool:
        #
    
