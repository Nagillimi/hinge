import numpy as np
from anatomical_hinge_nagillimi.constants import Constants
from anatomical_hinge_nagillimi.calibration.solution_set import SolutionSet
from anatomical_hinge_nagillimi.calibration.x_sphere import XSphere
from anatomical_hinge_nagillimi.calibration.motion_data import MotionData

class GradientDescent:
    def __init__(self):
        # Working iteration index
        self.k = int(0)

        # Solution iteration index
        self.s = int(0)

        # Residual errors for gyro and accel, e(x)
        # [(2)Nx1]
        self.err = []

        # Working sum of squares error
        #
        # [1] (used as [N] for threshold calc)
        self.sumOfSquares = []

        # Working solutions throughout algorithm iterations.
        # List not delcared emply, since there needs to be an initial guess
        # [4xN]
        self.x = [XSphere()]

        # Combined gyro & accel polar gradient (jacobian) for gradient descent.
        #
        # [2Nx4] & [Nx4] for axis & pose calibrations respectively.
        self.jac = []

        # Transpose of the combined gyro & accel polar gradient (jacobian) for
        # gradient descent.
        #
        # [4x2N] & [4xN] for axis & pose calibrations respectively.
        self.jacT = []

        # Hessian matrix (square), H = inv(Jac^T(x) * Jac(x))
        # 
        # [4x4] = [4x2N][2Nx4]
        self.hessian = np.zeros(shape=(4, 4))
        
        # Step direction for x based on the polar gradient and residual errors
        #
        # [4x1]
        self.stepDir = XSphere()

        # Step length based on the backtracking line search
        self.stepSize = 0.0

        # The derivative of the error at the current iteration, i.e. the new jacobian row(s).
        # de_dx = d(j | o)_dx * de_d(j | o)
        # 
        # [4x1] = [4x6][6x1]
        self.de_dx = np.zeros(shape=(4, 1))

        # Motion data stored for computing the pose calibration (requires the same data)
        self.motionData = MotionData()
        
        # Scratch/temp variables, for saving storage
        self.v3temp1 = np.zeros(shape=(1, 3))
        self.v3temp2 = np.zeros(shape=(1, 3))
        self.v3temp3 = np.zeros(shape=(1, 3))
        self.v3temp4 = np.zeros(shape=(1, 3))

        # single solution collection for the position algorithm convergence
        self.sols = []

    # Updates the step direction based on the polar gradient and residual errors.
    def updateStepDirection(self):
        # Hessain approx = inv(Jac^T(x) * Jac(x))
        # [4x4] = [4x(2)N][(2)Nx4]
        self.hessian = np.linalg.pinv(np.matmul(self.jacT, self.jac))

        # step dir = hessian * jacT * err
        # [4x1] = [4x4][4x(2)N][(2)Nx1]
        stepDirAsMatrix = np.matmul(self.hessian, np.matmul(self.jacT, self.err))
        self.stepDir = XSphere(
            t1 = stepDirAsMatrix[0],
            p1 = stepDirAsMatrix[1],
            t2 = stepDirAsMatrix[2],
            p2 = stepDirAsMatrix[3])


    # Updates the step length in a Gauss-Newton gradient descent algorithm using 
    # backtracking line search (BLS).
    #
    # This is a higher order function to give access to searching against the existing
    # error data
    def updateStepSize(self, getSumOfSquaresError):
        # gradient for BLS, "the change in vSOS" from the 2020 paper
        gradient = self.sumOfSquares[-1]
        if len(self.sumOfSquares) > 1:
            gradient = self.sumOfSquares[-1] - self.sumOfSquares[-2]

        # recompute gradient on first case, k = 0
        if(self.k is 0):
            gradient = self.sumOfSquares[-1]

        # intial step size, iterable
        self.stepSize = 1.0
        x = self.x[-1].getAsList()
        d = self.stepDir.getAsList()

        # initialize the conditional for the first BLS iteration
        leftSide: XSphere = XSphere(
            t1 = x[0] - (self.stepSize * d[0]),
            p1 = x[1] - (self.stepSize * d[1]),
            t2 = x[2] - (self.stepSize * d[2]),
            p2 = x[3] - (self.stepSize * d[3]))
        
        rightSide: XSphere = XSphere(
            t1 = x[0] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[0]),
            p1 = x[1] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[1]),
            t2 = x[2] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[2]),
            p2 = x[3] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[3]))

        # vSOS(current x - t * stepDir) > vSOS(current x - alpha * t * grad * stepDir)
        while (getSumOfSquaresError(leftSide) > getSumOfSquaresError(rightSide)):
            # decrement step size
            self.stepSize = Constants.BLS_BETA * self.stepSize

            # re-compute the conditionals with the new step size for evaluation
            leftSide = XSphere(
                t1 = x[0] - (self.stepSize * d[0]),
                p1 = x[1] - (self.stepSize * d[1]),
                t2 = x[2] - (self.stepSize * d[2]),
                p2 = x[3] - (self.stepSize * d[3]))
            
            rightSide = XSphere(
                t1 = x[0] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[0]),
                p1 = x[1] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[1]),
                t2 = x[2] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[2]),
                p2 = x[3] - (Constants.BLS_ALPHA * self.stepSize * gradient * d[3]))
            