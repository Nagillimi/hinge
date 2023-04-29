from numpy import array
from calibration.x_sphere import XSphere
from sensor_collection import SensorCollection

class CommonGradientDescent:
    # working iteration index
    k = 0

    # Solution iteration index
    s = 0

    # [1] (used as [N] for threshold calc) working sum of squares error
    vSOS = array(dtype=float)

    # [4xN] the working solutions throughout algorithm iterations
    x = array(dtype=XSphere)

    # the combined gyro & accel polar gradient (jacobian) for gradient descent
    jac = array(dtype=float)

    # the transpose of the combined gyro & accel polar gradient (jacobian) for
    # gradient descent
    jacT = array(dtype=float, ndmin=4)

    # [4x4] the hessian matrix, always square
    hessian = array(dtype=float, ndmin=4)
    
    # [4x1] step direction for x based on the polar gradient and residual errors
    stepDir = XSphere()

    # step length based on the backtracking line search
    stepSize = 0.0

    # scratch/temp variables, for saving storage
    v3temp1 = array(dtype=float)
    v3temp2 = array(dtype=float)
    v3temp3 = array(dtype=float)
    v3temp4 = array(dtype=float)
    jac_temp = array(dtype=float)

    # motion data stored for computing the pose calibration (requires the same data)
    motionData: array(dtype=SensorCollection)