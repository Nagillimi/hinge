import numpy as np
from calibration.x_sphere import XSphere
from constants import Constants
from motion_data import MotionData
from pose.error_function import ErrorFunction
from solution_set import SolutionSet
from axis.axis import AxisCalibration
from sensor_collection import SensorCollection

# calculates the o vectors
class PoseCalibration(ErrorFunction):
    def __init__(self):
        super().__init__()

        # estimates for j1 & j2 found from the hinge joint axis algorithm
        self.j1 = np.zeros(shape=(1, 3))
        self.j2 = np.zeros(shape=(1, 3))

        # scratch/temp variables, for saving storage
        self.o1_temp = np.zeros(shape=(1, 3))
        self.o2_temp = np.zeros(shape=(1, 3))


    def setAxis(self, axis: AxisCalibration):
        self.j1 = axis.sols[-1].j1
        self.j2 = axis.sols[-1].j2


    def update(self, data: MotionData):
        for _ in data.sensorData:
            # get current o vectors
            self.updatePoseVectors()

            # update vector e(x)
            # [Nx1]
            self.updateErrorFunction()

            # update matrix de_dx (polar gradient)
            # [Nx4]
            self.updateJacobian()

            # update the GD step direction
            self.updateStepDirection()

            # update the GD step size
            self.updateStepSize()

            # save new x
            self.x.append(XSphere(
                t1 = self.x[-1].imu1.theta - self.stepSize * self.stepDir.imu1.theta,
                p1 = self.x[-1].imu1.phi   - self.stepSize * self.stepDir.imu1.phi,
                t2 = self.x[-1].imu2.theta - self.stepSize * self.stepDir.imu2.theta,
                p2 = self.x[-1].imu2.phi   - self.stepSize * self.stepDir.imu2.phi
            ))
            
            # iterate
            self.k += 1

            # update cost function
            self.updateSumOfSquaresError()

            if (self.sumOfSquares[-1] - self.sumOfSquares[-2]) < Constants.MAXIMUM_POSITION_COST_ROC_THRESHOLD:
                # add good data to sols array
                self.sols.append(SolutionSet(
                    vSOS  = self.sumOfSquares[-1],
                    dvSOS = self.sumOfSquares[-1] - self.sumOfSquares[-2],
                    x     = self.x[-1]
                ))

                # iterate solution object
                self.s += 1
            