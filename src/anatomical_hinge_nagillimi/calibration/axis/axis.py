from typing import Union
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.sensor_collection import SensorCollection
from anatomical_hinge_nagillimi.result.kinematics_result import KinematicResult
from anatomical_hinge_nagillimi.result.calibration_result import CalibrationResult
from anatomical_hinge_nagillimi.calibration.x_sphere import XSphere
from anatomical_hinge_nagillimi.calibration.solution_set import SolutionSet
from anatomical_hinge_nagillimi.calibration.axis.error_function import ErrorFunction

# calculates the j vectors
class AxisCalibration(ErrorFunction):
    def __init__(self):
        super().__init__()
        self.isInitialized = False


    # Run calibration based on new sensor data from the collection
    def update(self, collection: SensorCollection) -> Union[CalibrationResult, KinematicResult]:
        self.motionData.update(collection)

        # get the first sse
        if self.isInitialized is False:
            self.updateSumOfSquaresError()
            self.isInitialized = True
            return CalibrationResult.INITIALIZED

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

        # update the GD step size with a SSE search
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

        # print iteration results
        # self.printCurrentIteration()

        if self.derivSumOfSquares < Constants.MAXIMUM_AXIS_COST_ROC_THRESHOLD:
            # add good data to sols array
            self.sols.append(SolutionSet(
                sumOfSquares = self.sumOfSquares.current,
                derivSumOfSquares = self.derivSumOfSquares,
                x = self.x[-1]
            ))

            # iterate solution object
            self.s += 1
            print("Axis solution iteration =", self.s)
            
            if self.s == Constants.SAMPLES_OF_UNIQUE_MOTION:
                self.updateFinalSolution()
                self.printFinalSolution()
                return CalibrationResult.SUCCESS
            else:
                return CalibrationResult.NOT_ENOUGH_UNIQUE_MOTION
        else:
            return CalibrationResult.MOTION_ERROR_ABOVE_THRESHOLD


    def printCurrentIteration(self):
        print("Cost function V(x) =", self.sumOfSquares.current)
        print("Cost function gradient dV(x) =", self.derivSumOfSquares)
        print("j-vectors = ", '\n'
           , '\t\t', "j1 =", self.j1_temp, '\n'
           , '\t\t', "j2 =", self.j2_temp
        )

    def printFinalSolution(self):
        print("\nFinal Solution Set")
        print("\tJ1 =", self.finalSolutionSet.x.vector1.toRectangular())
        print("\tJ2 =", self.finalSolutionSet.x.vector2.toRectangular())
        print("\n")
