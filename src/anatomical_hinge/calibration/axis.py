import numpy as np
from numpy import array
from calibration.x_sphere import XSphere
from calibration.common import CommonGradientDescent
from calibration.solution_set import AxisSolutionSet
from sensor_collection import SensorCollection

# calculates the j vectors
class AxisCalibration(CommonGradientDescent):
    # [2Nx1] residual errors for gyro and accel
    err = array(dtype=float)

    # Array of converged solutions for evaluation
    sols = array(dtype=AxisSolutionSet)

    # vector for joint CS, where c is not parallel to either j1 or j2
    c = array(dtype=float)

    # vector for joint CS, where x1 is orthogonal to j1 and y1
    x1 = array(dtype=float)

    # vector for joint CS, where y1 is orthogonal to j1 and x1
    y1 = array(dtype=float)

    # vector for joint CS, where x2 is orthogonal to j2 and y2
    x2 = array(dtype=float)

    # vector for joint CS, where y2 is orthogonal to j2 and x2
    y2 = array(dtype=float)

    # scratch/temp variables, for saving storage
    j1_temp = array(dtype=float)
    j2_temp = array(dtype=float)

    # def __init__(self) -> None:
    #     super().__init__()
    #     self.motionData = array(dtype=SensorCollection)

    def calibrate(self, collection: SensorCollection) -> bool:
        self.motionData.append(collection)

        return True
    
