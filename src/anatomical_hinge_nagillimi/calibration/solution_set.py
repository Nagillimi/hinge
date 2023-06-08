from anatomical_hinge_nagillimi.calibration.x_sphere import XSphere

class SolutionSet:
    def __init__(
            self,
            sumOfSquares: float,
            derivSumOfSquares: float,
            x: XSphere
    ):
        # the converged accumulation of sum of squares error (cost)
        self.sumOfSquares = sumOfSquares

        # the final derivative of the sum of squares error (dcost)
        self.derivSumOfSquares = derivSumOfSquares

        # the converged solution set
        self.x = x
