from numpy import array
from calibration.x_sphere import XSphere
from calibration.data_buffer import DataBuffer
from calibration.solution_set import AxisSolutionSet, PoseSolutionSet

class StateVector:
    # Useful databuffer containing sensor readings during algorithm runtime
    # for convergence evaluation
    dataBuffer = DataBuffer()

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
    

class AxisStateVector(StateVector):
    # [2Nx1] residual errors for gyro and accel
    err = array(dtype=float)

    # Array of converged solutions for evaluation
    sols = array(dtype=AxisSolutionSet)

    # vector for joint CS, where c is not parallel to either j1 or j2
    c = array(dtype=float)

    # vector for joint CS, where x1 is orthogonal to j1 and y1
    x1 = array(dtype=float)

    # vector for joint CS, where y1 is orthogonal to j1 and x1
    y1 = array(dtype=float)

    # vector for joint CS, where x2 is orthogonal to j2 and y2
    x2 = array(dtype=float)

    # vector for joint CS, where y2 is orthogonal to j2 and x2
    y2 = array(dtype=float)

    # scratch/temp variables, for saving storage
    j1_temp = array(dtype=float)
    j2_temp = array(dtype=float)


class PoseStateVector(StateVector):
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
