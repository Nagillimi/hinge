from typing import Optional
import numpy as np
from calibration.gradient_descent import GradientDescent
from calibration.x_sphere import XSphere

class ErrorFunction(GradientDescent):
    def __init__(self):
        super().__init__()

        # scratch/temp variables, for saving storage
        self.o1_temp = np.zeros(shape=(1, 3))
        self.o2_temp = np.zeros(shape=(1, 3))

    # updates the error function
    # [Nx1]
    def updateErrorFunction(self):
        self.err.append(self.getIMU1Error() - self.getIMU2Error())


    # Updates the temporary joint vectors based on the current x
    # [3x1]
    def updatePoseVectors(self):
        self.o1_temp = self.x[-1].vector1.toRectangular()
        self.o2_temp = self.x[-1].vector2.toRectangular()


    # Updates the accumulating SOS error
    # [Nx1]
    def updateSumOfSquaresError(self):
        self.sumOfSquares.append(self.getSumSquaresError())


    # Updates the jacobian (de_dx) for the current iteration
    # [Nx4]
    def updateJacobian(self):
        sensorData = self.motionData.sensorData[-1]
        xData = self.x[-1]
        s = xData.sin()
        c = xData.cos()

        # compute the dj_dx based on current x, [4x6]
        do_dx = np.array([
            [-(s[0]*c[1]), -(s[0]*s[1]),   +(c[0]),     (0),          (0),        (0)  ],
            [-(c[0]*s[1]), +(c[0]*c[1]),      (0),      (0),          (0),        (0)  ],
            [     (0),          (0),          (0), -(s[2]*c[3]), -(s[2]*s[3]),  +(c[2])],
            [     (0),          (0),          (0), -(c[2]*s[3]), +(c[2]*c[3]),    (0)  ]
        ])

        # construct the de_do's
        de_do1 = self.getRadialAndTangentialAcceleration(
            sensorData.g1.raw.current,
            sensorData.a1.raw.current - self.getRadialAndTangentialAcceleration(
                sensorData.g1.raw.current, self.o1_temp, sensorData.g1.deriv.current
            ),
            sensorData.g1.deriv.current
        )
        de_do1 / np.linalg.norm(de_do1)
        de_do2 = self.getRadialAndTangentialAcceleration(
            sensorData.g2.raw.current,
            sensorData.a2.raw.current - self.getRadialAndTangentialAcceleration(
                sensorData.g2.raw.current, self.o2_temp, sensorData.g2.deriv.current
            ),
            sensorData.g2.deriv.current
        )
        de_do2 / np.linalg.norm(de_do2)


        # single accel-based de_dx = dj_dx * de_dj, [4x1] = [4x6][6x1]
        # inserted at the end of the jacbian row index (N)
        self.jac.append(
            np.matmul(do_dx, np.vstack(de_do1, de_do2))# * Constants.wACCEL
        )
        self.jacT.append(
            np.matmul(do_dx, np.hstack(de_do1, de_do2))# * Constants.wACCEL
        )


    # Gets the current iteration of the sum of squares error
    # if x is passed, enter search mode which returns a use case
    def getSumSquaresError(self, x = Optional[XSphere]) -> float:
        return sum(self.sumOfSquares) + (self.getIMU1Error(x) - self.getIMU2Error(x))**2


    def getIMU1Error(self, x = Optional[XSphere]) -> float:
        sensorData = self.motionData.sensorData[-1]
        o1 = self.o1_temp
        if x:
            o1 = x.vector1.toRectangular()
        error = np.array(sensorData.a1.raw.current) - self.getRadialAndTangentialAcceleration(
            sensorData.g1.raw.current, o1, sensorData.g1.deriv.current
        )
        return np.linalg.norm(error)


    def getIMU2Error(self, x = Optional[XSphere]) -> float:
        sensorData = self.motionData.sensorData[-1]
        o2 = self.o2_temp
        if x:
            o2 = x.vector2.toRectangular()
        error = np.array(sensorData.a2.raw.current) - self.getRadialAndTangentialAcceleration(
            sensorData.g2.raw.current, o2, sensorData.g2.deriv.current
        )
        return np.linalg.norm(error)
        
    # Gets the radial and tangential acceleration for a single IMU
    # Follows a pure functional design since it's used in many circumstances
    def getRadialAndTangentialAcceleration(
            self,
            subVar: list[float],
            arg: list[float],
            subVarDot: list[float],
            transpose = False):
        if transpose is True:
            return np.cross(np.cross(arg, subVar), subVar) + np.cross(arg, subVarDot)
        return np.cross(subVar, np.cross(subVar, arg)) + np.cross(subVarDot, arg)
