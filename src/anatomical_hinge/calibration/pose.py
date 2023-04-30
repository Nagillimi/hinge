import numpy as np
from common import CommonGradientDescent
from solution_set import PoseSolutionSet
from axis import AxisCalibration
from sensor_collection import SensorCollection

# calculates the o vectors
class PoseCalibration(CommonGradientDescent):
    # Residual errors for gyro and accel
    #
    # [Nx1]
    err = [float()]

    # single solution collection for the position algorithm convergence
    sol: PoseSolutionSet

    # estimates for j1 & j2 found from the hinge joint axis algorithm
    j1 = np.zeros(shape=(1, 3))
    j2 = np.zeros(shape=(1, 3))

    # scratch/temp variables, for saving storage
    o1_temp = np.zeros(shape=(1, 3))
    o2_temp = np.zeros(shape=(1, 3))
    
    def setAxis(self, axis: AxisCalibration):
        self.j1 = axis.sols[-1].j1
        self.j2 = axis.sols[-1].j2

    def calibrate(self, data: list[SensorCollection]):
        self.motionData = data
