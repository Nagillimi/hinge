from typing import Union
from constants import Constants
from result.kinematics_result import KinematicResult
from result.calibration_result import CalibrationResult
from calibration.x_sphere import XSphere
from calibration.solution_set import SolutionSet
from sensor_collection import SensorCollection
from error_function import ErrorFunction

# calculates the j vectors
class AxisCalibration(ErrorFunction):
    def __init__(self):
        super().__init__()


    # Run calibration based on new sensor data from the collection
    def update(self, collection: SensorCollection) -> Union[CalibrationResult, KinematicResult]:
        self.motionData.update(collection)

        # only run calibration on signals with consistent motion
        motionTest = self.motionData.kinematic.testForConsecutive()
        if motionTest != KinematicResult.CONSECUTIVE_MOTION_DETECTED:
            return motionTest
        
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
            t1 = self.x[-1].vector1.theta - self.stepSize * self.stepDir.vector1.theta,
            p1 = self.x[-1].vector1.phi   - self.stepSize * self.stepDir.vector1.phi,
            t2 = self.x[-1].vector2.theta - self.stepSize * self.stepDir.vector2.theta,
            p2 = self.x[-1].vector2.phi   - self.stepSize * self.stepDir.vector2.phi
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
