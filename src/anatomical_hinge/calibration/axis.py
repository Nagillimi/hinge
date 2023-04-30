import numpy as np
from enum import Enum
from error.calibration_error import CalibrationError
from calibration.x_sphere import XSphere
from calibration.common import CommonGradientDescent
from calibration.solution_set import AxisSolutionSet
from sensor_collection import SensorCollection


# calculates the j vectors
class AxisCalibration(CommonGradientDescent):
    # Residual errors for gyro and accel, e(x)
    # [2Nx1]
    err = [float()]

    # Array of converged solutions for evaluation
    sols = [AxisSolutionSet()]

    # vector for joint CS, where c is not parallel to either j1 or j2
    c = np.array((0, 1, 0))

    # vector for joint CS, where x1 is orthogonal to j1 and y1
    x1 = np.zeros(shape=(1, 3))

    # vector for joint CS, where y1 is orthogonal to j1 and x1
    y1 = np.zeros(shape=(1, 3))

    # vector for joint CS, where x2 is orthogonal to j2 and y2
    x2 = np.zeros(shape=(1, 3))

    # vector for joint CS, where y2 is orthogonal to j2 and x2
    y2 = np.zeros(shape=(1, 3))

    # scratch/temp variables, for saving storage
    j1_temp = np.zeros(shape=(1, 3))
    j2_temp = np.zeros(shape=(1, 3))

    def calibrate(self, collection: SensorCollection) -> CalibrationError:
        self.motionData.append(collection)

        # get current j vectors (temp)
        self.j1_temp = self.x[-1].imu1.toRectangular()
        self.j2_temp = self.x[-1].imu2.toRectangular()

        # compute vector e(x)
        

        # compute matrix de_dx (polar gradient)


        # compute the GD step direction


        # compute the GD step size
        

        return True
    
    def computeGyroError():
        pass

    def computeAccelError():
        pass
