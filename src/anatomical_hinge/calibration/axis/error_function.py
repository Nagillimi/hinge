from typing import Optional
import numpy as np
from gradient_descent import GradientDescent
from calibration.x_sphere import XSphere
from constants import Constants

class ErrorFunction(GradientDescent):
    def __init__(self):
        super().__init__()

        # scratch/temp variables, for saving storage
        self.j1_temp = np.zeros(shape=(1, 3))
        self.j2_temp = np.zeros(shape=(1, 3))

    # updates the error function
    # [2Nx1]
    def updateErrorFunction(self):
        self.err.insert(self.k, self.getGyroError())
        self.err.append(self.getAccelError())


    # Updates the temporary joint vectors based on the current x
    # [3x1]
    def updateJointVectors(self):
        self.j1_temp = self.x[-1].vector1.toRectangular()
        self.j2_temp = self.x[-1].vector2.toRectangular()


    # Updates the accumulating SOS error
    # [Nx1]
    def updateSumOfSquaresError(self):
        self.sumOfSquares.append(self.getSumSquaresError())
        

    # Updates the jacobian (de_dx) for the current iteration
    # [2Nx4]
    def updateJacobian(self):
        sensorData = self.motionData.sensorData[-1]
        xData = self.x[-1]
        s = xData.sin()
        c = xData.cos()

        # compute the dj_dx based on current x, [4x6]
        dj_dx = np.array([
            [-(s[0]*c[1]), -(s[0]*s[1]),   +(c[0]),     (0),          (0),        (0)  ],
            [-(c[0]*s[1]), +(c[0]*c[1]),      (0),      (0),          (0),        (0)  ],
            [     (0),          (0),          (0), -(s[2]*c[3]), -(s[2]*s[3]),  +(c[2])],
            [     (0),          (0),          (0), -(c[2]*s[3]), +(c[2]*c[3]),    (0)  ]
        ])

        # single gyro-based de_dx = dj_dx * de_dj, [4x1] = [4x6][6x1]
        # inserted halfway down the jacbian row index (N)
        g1_de_dj = np.cross(sensorData.g1, self.j1_temp)
        norm = np.linalg.norm(g1_de_dj)
        g1_de_dj = np.cross(g1_de_dj, sensorData.g1) / norm
        g2_de_dj = np.cross(sensorData.g2, self.j2_temp)
        norm = np.linalg.norm(g2_de_dj)
        g2_de_dj = np.cross(g2_de_dj, sensorData.g2) / norm
        self.jac.insert(
            self.k,
            np.matmul(dj_dx, np.vstack(g1_de_dj, -g2_de_dj)) * Constants.wGYRO
        )
        self.jacT.insert(
            self.k,
            np.matmul(dj_dx, np.hstack(g1_de_dj, -g2_de_dj)) * Constants.wGYRO
        )

        # single accel-based de_dx = dj_dx * de_dj, [4x1] = [4x6][6x1]
        # inserted at the end of the jacbian row index (2N)
        self.jac.append(
            np.matmul(dj_dx, np.vstack(sensorData.a1, -sensorData.a2)) * Constants.wACCEL
        )
        self.jacT.append(
            np.matmul(dj_dx, np.hstack(sensorData.a1, -sensorData.a2)) * Constants.wACCEL
        )
        

    # Gets the current iteration of the sum of squares error
    # if x is passed, enter search mode which returns a use case
    def getSumSquaresError(self, x = Optional[XSphere]) -> float:
        if x:
            return (
                sum(self.sumOfSquares) +
                self.getGyroError(x)**2 +
                self.getAccelError(x)**2
            )
        
        return (
            sum(self.sumOfSquares) +
            self.getGyroError()**2 +
            self.getAccelError()**2
        )


    def getGyroError(self, x = Optional[XSphere]) -> float:
        sensorData = self.motionData.sensorData[-1]
        if x:
            j1_temp = x.vector1.toRectangular()
            j2_temp = x.vector2.toRectangular()
            self.v3temp1 = np.cross(sensorData.g1, j1_temp)
            self.v3temp2 = np.cross(sensorData.g2, j2_temp)
        else:
            self.v3temp1 = np.cross(sensorData.g1, self.j1_temp)
            self.v3temp2 = np.cross(sensorData.g2, self.j2_temp)

        c1 = np.linalg.norm(self.v3temp1)
        c2 = np.linalg.norm(self.v3temp2)

        return Constants.wGYRO * (c1 - c2)


    def getAccelError(self, x = Optional[XSphere]) -> float:
        sensorData = self.motionData.sensorData[-1]
        if x:
            j1_temp = x.vector1.toRectangular()
            j2_temp = x.vector2.toRectangular()
            c1 = j1_temp.dot(sensorData.a1)
            c2 = j2_temp.dot(sensorData.a2) 
        else:
            c1 = self.j1_temp.dot(sensorData.a1)
            c2 = self.j2_temp.dot(sensorData.a2)       
        
        return Constants.wACCEL * (c1 - c2)
    