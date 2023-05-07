from typing import Optional
import numpy as np
from gradient_descent import GradientDescent
from calibration.x_sphere import XSphere
from constants import Constants

class ErrorFunction(GradientDescent):
    def __init__(self):
        super().__init__()

        # scratch/temp variables, for saving storage
        self.o1_temp = np.zeros(shape=(1, 3))
        self.o2_temp = np.zeros(shape=(1, 3))

    # updates the error function
    # [Nx1]
    def updateErrorFunction(self):
        pass


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

        # construct the de_do


        # single accel-based de_dx = dj_dx * de_dj, [4x1] = [4x6][6x1]
        # inserted at the end of the jacbian row index (N)
        self.jac.append(
            np.matmul(do_dx, np.vstack(     )) * Constants.wACCEL
        )
        self.jacT.append(
            np.matmul(do_dx, np.hstack(     )) * Constants.wACCEL
        )
        

    # Gets the current iteration of the sum of squares error
    # if x is passed, enter search mode which returns a use case
    def getSumSquaresError(self, x = Optional[XSphere]) -> float:
        if x:
            return (
                sum(self.sumOfSquares) 
            )
        
        return (
            sum(self.sumOfSquares) 
        )


    def getIMU1Error(self, x = Optional[XSphere]) -> float:
        sensorData = self.motionData.sensorData[-1]
        if x:
            o1_temp = x.vector1.toRectangular()
        else:
            pass


    def getIMU2Error(self, x = Optional[XSphere]) -> float:
        sensorData = self.motionData.sensorData[-1]
        if x:
            o1_temp = x.vector1.toRectangular()
        else:
            pass
        
    
    def getRadialAndTangentialAcceleration(self) -> np.NDArray[float]:
        pass
