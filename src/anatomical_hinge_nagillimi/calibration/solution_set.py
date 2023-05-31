from calibration.x_sphere import XSphere

class SolutionSet:
    def __init__(
            self,
            vSOS: float,
            dvSOS: float,
            x: XSphere
    ):
        # the converged accumulation of sum of squares error (cost)
        self.vSOS = vSOS

        # the final derivative of the sum of squares error (dcost)
        self.dvSOS = dvSOS

        # the converged solution set
        self.x = x
