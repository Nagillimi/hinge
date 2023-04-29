from numpy import array
from calibration.x_sphere import XSphere

class SolutionSet:
    # the converged accumulation of sum of squares error (cost)
    vSOS = 0.0
    # the final derivative of the sum of squares error (dcost)
    dvSOS = 0.0
    # the converged solution set
    x = XSphere()

    def set(
            self,
            vSOS: float,
            dvSOS: float,
            x: XSphere) -> None:
        self.vSOS = vSOS
        self.dvSOS = dvSOS
        self.x = x

class AxisSolutionSet(SolutionSet):
    j1: array(dtype=float)
    j2: array(dtype=float)

class PoseSolutionSet(SolutionSet):
    o1: array(dtype=float)
    o2: array(dtype=float)
