from typing import List
import numpy as np
from anatomical_hinge_nagillimi.calibration.motion_data import MotionData
from anatomical_hinge_nagillimi.calibration.solution_set import SolutionSet
from anatomical_hinge_nagillimi.calibration.x_sphere import XSphere
from anatomical_hinge_nagillimi.utilities.historic import HistoricNumber

class CommonElements:
    def __init__(self):
        # Working iteration index
        self.k = int(0)

        # Solution iteration index
        self.s = int(0)
        
        # Working SSE array
        #
        # [1] (used as [N] for threshold calc)
        self.sumOfSquares = HistoricNumber()

        # Derivate of the SSE
        self.derivSumOfSquares = 0.

        # Motion data stored for computing the pose calibration (requires the same data)
        self.motionData = MotionData()
        
        # single solution collection for the position algorithm convergence
        self.sols: List[SolutionSet] = []

        # the final solution set, to be used as the converged output, derived from the
        # collection of solution sets
        self.finalSolutionSet = SolutionSet(0,0,XSphere())


    # Updates the final solution based on the converged results
    def updateFinalSolution(self):
        finalTheta1 = []
        finalPhi1 = []
        finalTheta2 = []
        finalPhi2 = []
        for sol in self.sols:
            finalTheta1.append(sol.x.vector1.theta)
            finalPhi1.append(sol.x.vector1.phi)
            finalTheta2.append(sol.x.vector2.theta)
            finalPhi2.append(sol.x.vector2.phi)

        self.finalSolutionSet = SolutionSet(
            sumOfSquares = self.sols[-1].sumOfSquares,
            derivSumOfSquares = self.sols[-1].derivSumOfSquares,
            x = XSphere(
                t1 = np.median(finalTheta1),
                p1 = np.median(finalPhi1),
                t2 = np.median(finalTheta2),
                p2 = np.median(finalPhi2),
            )
        )
