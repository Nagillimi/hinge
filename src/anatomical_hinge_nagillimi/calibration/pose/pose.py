import numpy as np
from anatomical_hinge_nagillimi.calibration.x_sphere import XSphere
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.calibration.motion_data import MotionData
from anatomical_hinge_nagillimi.calibration.pose.error_function import ErrorFunction
from anatomical_hinge_nagillimi.calibration.solution_set import SolutionSet
from anatomical_hinge_nagillimi.kinematics import Kinematic
from anatomical_hinge_nagillimi.result.kinematics_result import KinematicResult

# calculates the o vectors
class PoseCalibration(ErrorFunction):
    def __init__(self):
        super().__init__()


    def setAxis(self, axisSolution: SolutionSet):
        self.j1 = np.array(axisSolution.x.vector1.toRectangular())
        self.j2 = np.array(axisSolution.x.vector2.toRectangular())


    def update(self, data: MotionData):
        length = len(data.sensorData)
        for newCollection in data.sensorData:
            # update saved motion data by iteration
            self.motionData.update(newCollection)

            motionTest = data.kinematic.testForConsecutive()
            if motionTest != KinematicResult.CONSECUTIVE_MOTION_DETECTED: continue

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

            if self.derivSumOfSquares < Constants.MAXIMUM_POSITION_COST_ROC_THRESHOLD:
                # add good data to sols array
                self.sols.append(SolutionSet(
                    sumOfSquares = self.sumOfSquares.current,
                    derivSumOfSquares = self.derivSumOfSquares,
                    x = self.x[-1]
                ))

                # iterate solution object
                self.s += 1
                print("Pose solution iteration =", self.s)

        self.updateFinalSolution()
        self.printFinalSolution()
            
    def printCurrentIteration(self):
        print("Cost function V(x) =", self.sumOfSquares.current)
        print("Cost function gradient dV(x) =", self.derivSumOfSquares)
        print("o-vectors = ", '\n'
           , '\t\t', "o1 =", self.o1_temp, '\n'
           , '\t\t', "o2 =", self.o2_temp
        )

    def printFinalSolution(self):
        print("\nFinal Solution Set")
        print("\tO1 =", self.finalSolutionSet.x.vector1.toRectangular())
        print("\tO2 =", self.finalSolutionSet.x.vector2.toRectangular())
        print("\n")
