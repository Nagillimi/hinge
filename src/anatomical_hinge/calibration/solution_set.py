from numpy import array
from enum import Enum
from calibration.x_sphere import XSphere

class SolutionTypeNames(Enum):
    J_VECTORS = 'j-vectors'
    O_VECTORS = 'o-vectors'

class SolutionSet:
    # the associated type of set
    name: SolutionTypeNames
    # the converged accumulation of sum of squares error (cost)
    vSOS: float
    # the final derivative of the sum of squares error (dcost)
    dvSOS: float
    # the converged solution set
    x: XSphere

    def __init__(self, name: SolutionTypeNames) -> None:
        self.name = name
        self.set(
            vSOS=0,
            dvSOS=0,
            x=XSphere()
        )

    def set(
            self,
            vSOS: float,
            dvSOS: float,
            x: XSphere) -> None:
        self.vSOS = vSOS
        self.dvSOS = dvSOS
        self.x = x

class AxisSolutionSet(SolutionSet):
    name = SolutionTypeNames.J_VECTORS
    j1: array(dtype=float)
    j2: array(dtype=float)

class PoseSolutionSet(SolutionSet):
    name = SolutionTypeNames.O_VECTORS
    o1: array(dtype=float)
    o2: array(dtype=float)
