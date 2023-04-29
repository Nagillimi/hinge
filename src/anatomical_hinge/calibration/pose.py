from numpy import array
from calibration.common import CommonGradientDescent
from calibration.solution_set import PoseSolutionSet
from sensor_collection import SensorCollection

# calculates the o vectors
class PoseCalibration(CommonGradientDescent):
    # [Nx1] residual errors for gyro and accel
    err = array(dtype=float)

    # single solution collection for the position algorithm convergence
    sol: PoseSolutionSet

    # estimates for j1 & j2 found from the hinge joint axis algorithm
    j1_est = array(dtype=float)
    j2_est = array(dtype=float)

    # scratch/temp variables, for saving storage
    o1_temp = array(dtype=float)
    o2_temp = array(dtype=float)
    
    def calibrate(self, data: array(dtype=SensorCollection)):
        self.motionData = data
