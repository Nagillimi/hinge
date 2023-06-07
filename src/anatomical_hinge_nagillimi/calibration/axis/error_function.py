from typing import Optional
import numpy as np
from anatomical_hinge_nagillimi.calibration.gradient_descent import GradientDescent
from anatomical_hinge_nagillimi.calibration.x_sphere import XSphere
from anatomical_hinge_nagillimi.constants import Constants

class ErrorFunction(GradientDescent):
    def __init__(self):
        super().__init__()

        # scratch/temp variables, for saving storage
        self.j1_temp = np.zeros(shape=(1, 3), dtype=float)
        self.j2_temp = np.zeros(shape=(1, 3), dtype=float)

    # updates the error function
    # [2Nx1]
    def updateErrorFunction(self):
        self.err.insert(self.k, self.getGyroError())
        self.err.append(self.getAccelError())


    # Updates the temporary joint vectors based on the current x
    # [3x1]
    def updateJointVectors(self):
        self.j1_temp = np.array(self.x[-1].vector1.toRectangular())
        self.j2_temp = np.array(self.x[-1].vector2.toRectangular())


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
            [     (0),          (0),          (0), -(c[2]*s[3]), +(c[2]*c[3]),    (0)  ]])

        # single gyro-based de_dx = dj_dx * de_dj, [4x1] = [4x6][6x1]
        # inserted halfway down the jacbian row index (N)
        g1_de_dj = np.cross(sensorData.g1.raw.current(), self.j1_temp)
        norm = np.linalg.norm(g1_de_dj)
        g1_de_dj = np.cross(g1_de_dj, sensorData.g1.raw.current()) / norm
        g2_de_dj = np.cross(sensorData.g2.raw.current(), self.j2_temp)
        norm = np.linalg.norm(g2_de_dj)
        g2_de_dj = np.cross(g2_de_dj, sensorData.g2.raw.current()) / norm

        de_dj = np.hstack((g1_de_dj, -g2_de_dj))
        de_dx = np.ndarray.tolist(np.matmul(dj_dx, de_dj) * Constants.wGYRO)
        # for the first iteration, append the 2d array
        self.jac.insert(self.k, de_dx)
        if len(self.jacT) == 0:
            self.jacT = [[de_dx[0]], [de_dx[1]], [de_dx[2]], [de_dx[3]]]
        else:
            for i, de in enumerate(de_dx):
                self.jacT[i].insert(self.k, de)

        # single accel-based de_dx = dj_dx * de_dj, [4x1] = [4x6][6x1]
        # inserted at the end of the jacbian row index (2N)
        de_dj = np.hstack((sensorData.a1.raw.current(), sensorData.a2.raw.current()))
        de_dx = np.ndarray.tolist(np.matmul(dj_dx, de_dj) * Constants.wACCEL)
        self.jac.append(de_dx)
        for i, de in enumerate(de_dx):
            self.jacT[i].append(de)
        

    # Gets the current iteration of the sum of squares error
    # if x is passed, enter search mode which returns a use case
    def getSumSquaresError(self, x: Optional[XSphere] = None) -> float:
        pastSum = 0
        if len(self.sumOfSquares) > 0: pastSum = self.sumOfSquares[-1]

        if x: return pastSum + self.getGyroError(x)**2 + self.getAccelError(x)**2
        return pastSum + self.getGyroError()**2 + self.getAccelError()**2


    def getGyroError(self, x: Optional[XSphere] = None) -> float:
        sensorData = self.motionData.sensorData[-1]
        j1 = self.j1_temp
        j2 = self.j2_temp
        if x:
            j1 = np.array(x.vector1.toRectangular())
            j2 = np.array(x.vector2.toRectangular())
        c1 = np.linalg.norm(np.cross(sensorData.g1.raw.current(), j1))
        c2 = np.linalg.norm(np.cross(sensorData.g2.raw.current(), j2))
        return Constants.wGYRO * (c1 - c2)


    def getAccelError(self, x: Optional[XSphere] = None) -> float:
        sensorData = self.motionData.sensorData[-1]
        j1 = self.j1_temp
        j2 = self.j2_temp
        if x:
            j1 = np.array(x.vector1.toRectangular())
            j2 = np.array(x.vector2.toRectangular())
        c1 = j1.dot(sensorData.a1.raw.current())
        c2 = j2.dot(sensorData.a2.raw.current())       
        return Constants.wACCEL * (c1 - c2)
    