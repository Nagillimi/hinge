import numpy as np
from calibration.x_sphere import XSphere
from sensor_collection import SensorCollection

class CommonGradientDescent:
    # Working iteration index
    k = 0

    # Solution iteration index
    s = 0

    # Working sum of squares error
    #
    # [1] (used as [N] for threshold calc)
    vSOS = [float()]

    # Working solutions throughout algorithm iterations.
    #
    # [4xN]
    x = [XSphere()]

    # Combined gyro & accel polar gradient (jacobian) for gradient descent.
    #
    # [2Nx4] & [Nx4] for axis & pose calibrations respectively.
    jac = [[float()]]

    # Transpose of the combined gyro & accel polar gradient (jacobian) for
    # gradient descent.
    #
    # [4x2N] & [4xN] for axis & pose calibrations respectively.
    jacT = [[float()]]

    # Hessian matrix (square), H = inv(Jac^T(x) * Jac(x))
    # 
    # [4x4] = [4x2N][2Nx4]
    hessian = np.zeros(shape=(4, 4))
    
    # Step direction for x based on the polar gradient and residual errors
    #
    # [4x1]
    stepDir = XSphere()

    # Step length based on the backtracking line search
    stepSize = 0.0

    # The derivative of the error at the current iteration, ie the new jacobian row.
    # de_dx = d(j | o)_dx * de_d(j | o)
    # 
    # [4x1] = [4x6][6x1]
    de_dx = np.zeros(shape=(4, 1))

    # Motion data stored for computing the pose calibration (requires the same data)
    motionData = [SensorCollection()]
    
    # Scratch/temp variables, for saving storage
    v3temp1 = np.zeros(shape=(1, 3))
    v3temp2 = np.zeros(shape=(1, 3))
    v3temp3 = np.zeros(shape=(1, 3))
    v3temp4 = np.zeros(shape=(1, 3))
