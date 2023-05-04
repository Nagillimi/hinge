import numpy as np
from anatomical_hinge.constants import Constants
from result.calibration_result import CalibrationResult
from calibration.x_sphere import XSphere
from calibration.solution_set import SolutionSet
from sensor_collection import SensorCollection
from error_function import ErrorFunction

# calculates the j vectors
class AxisCalibration(ErrorFunction):
    def __init__(self):
        super()

        # Array of converged solutions for evaluation
        self.sols = [SolutionSet]

        # vector for joint CS, where x1 is orthogonal to j1 and y1
        self.x1 = np.zeros(shape=(1, 3))

        # vector for joint CS, where y1 is orthogonal to j1 and x1
        self.y1 = np.zeros(shape=(1, 3))

        # vector for joint CS, where x2 is orthogonal to j2 and y2
        self.x2 = np.zeros(shape=(1, 3))

        # vector for joint CS, where y2 is orthogonal to j2 and x2
        self.y2 = np.zeros(shape=(1, 3))


    # Run calibration based on new sensor data from the collection
    def calibrate(self, collection: SensorCollection) -> CalibrationResult:
        self.motionData.append(collection)

        # get current j vectors (temp)
        self.updateJointVectors()

        # update vector e(x)
        # [2Nx1]
        self.updateErrorFunction()

        # update matrix de_dx (polar gradient)
        # [2Nx4]
        self.updateJacobian()

        # update the GD step direction
        self.updateStepDirection()

        # update the GD step size
        self.updateStepSize(self.getSumSquaresError)

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

        if (self.sumOfSquares[-1] - self.sumOfSquares[-2]) < Constants.MAXIMUM_AXIS_COST_ROC_THRESHOLD:
            # add good data to sols array
            self.sols.append(SolutionSet(
                vSOS  = self.sumOfSquares[-1],
                dvSOS = self.sumOfSquares[-1] - self.sumOfSquares[-2],
                x     = self.x[-1]
            ))

            # iterate solution object
            self.s += 1
            
            if self.s == Constants.SAMPLES_OF_UNIQUE_MOTION:
                return CalibrationResult.SUCCESS
            else:
                return CalibrationResult.NOT_ENOUGH_UNIQUE_MOTION
        else:
            return CalibrationResult.MOTION_ERROR_ABOVE_THRESHOLD
