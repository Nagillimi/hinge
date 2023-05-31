# Collects the constants required to compute the necessary algorithms in obtaining
# a 1dof joint angle
# 
# TRY TO ONLY CHANGE ONE CONSTANT AT A TIME

import math

class Constants:
    # Switch to display logs to the console.  
    # Warning: very log heavy, not recommended.
    DISPLAY_DETAILS = False

    # Switch OFF/ON the WAITING_FOR_MOTION step
    WAIT_FOR_MOTION = False

    # The preferred baseline latency of the algorithm in milliseconds
    # 
    # 10ms => 100Hz  
    # 12ms => 83.3Hz  
    # 15ms => 66.7Hz  
    # 20ms => 50Hz
    OPTIMAL_ALGORITHM_LATENCY_MS = 12

    # DO NOT SET HERE!
    # 
    # The downsampling index which consistently discards data for the incoming imu signals
    # during algorithm computation.
    # 
    # Should only be set externally from a subclass which analyzes the incoming signal latency
    # to provide a rounded number of samples to best match the various algorithm incoming
    # latency requirements. Taken from Seel 2012/14 & 2020 articles, sampling can "use every third
    # to tenth sample" from "common sampling rates of 50Hz to 300Hz".
    # 
    # A value of 5 would mean to take every 5th sample when computing the algorithm.
    # 
    # Note: signals are only downsampled during algorithm computation, hinge angle will stream at
    # full rate once finished. 
    ALGORITHM_DOWNSAMPLING_INDEX = 0

    # Boolean to switch OFF/ON the initial condition setting from average accel data.
    # 
    # If ON (true)   =>    allows the average function to run which will take in N samples
    #                      (N = SAMPLE_COUNT_FOR_INITIAL_CONDITION) of accelerometer-based 
    #                      hinge angle and set the IC to the gyro & combined hinge angles.  
    # If OFF (false) =>    doesn't take average and just sets the most recent accelerometer-
    #                      based hinge to the IC for the gyro & combined hinge angles.  
    USE_AVG_ACCEL_IC = True

    # The number of samples to collet an average accelerometer-based hinge
    # angle from.
    # 
    # Determined Experimentally to be 10
    NUM_SAMPLES_AVG_ACCEL_IC = 10

    # a.k.a. "Kalman Gain".  
    # Gain of the sensory fusion equation in combining gyro
    # & accel data.  
    # 
    # Closer to 1 favours more the accelerometer-based angle.  
    # Close to 0 favours more the gyroscopic-based angle.
    # 
    # Determined experimentally to be 0.01
    SENSOR_FUSION_WEIGHT = 0.005

    # The common vector that translates each IMU sensor frame to the global
    # joint frame
    C_VECTOR = [0.0, 1.0, 0.0]

    #: HINGE JOINT AXIS & POSE ALGORITHM CONSTANTS 

    # The maximum size of the good motion data sample buffer, used to
    # accumulate the samples where the cost function for each algorithm
    # is below a certain threshold. An algorithm completes once a buffer
    # is filled to this size- like the "painting space" calibration in mocap
    # systems.
    SAMPLES_OF_UNIQUE_MOTION = 10

    # The maximum cost rate of change index used to parse out the good datasets
    # during the hinge axis algorithm. All samples with a cost at or below this
    # threshold will pass and be included in the working variable collection
    MAXIMUM_AXIS_COST_ROC_THRESHOLD = 0.5

    # The maximum cost rate of change index used to parse out the good datasets
    # during the hinge position algorithm. All samples with a cost at or below this
    # threshold will pass and be included in the working variable collection
    MAXIMUM_POSITION_COST_ROC_THRESHOLD = 0.005

    # DEPRECATED!
    # 
    # The minimum number of samples allowed to pass convergence, taken from
    # Seel 2012/14: where "N >> 4" samples.
    # 
    # Determined experimentally to be 10 samples
    MINIMUM_SAMPLES_FOR_CONVERGENCE = 10

    # DEPRECATED!
    # 
    # The minimum allowable tolerance required between the delta of sum of 
    # squares error V(x) iterations of gradient descent for the axis algorithm.
    # Used for j vectors only.
    # 
    # Determined experimentally to be 0.00000001
    AXIS_GRADDESC_TOL = 0.00000001

    # DEPRECATED!
    # 
    # The minimum allowable tolerance required between the delta of sum of 
    # squares error V(x) iterations of gradient descent for the position algorithm.
    # Used for o vectors only.  
    # 
    # Determined experimentally to be 0.00000001
    POSE_GRADDESC_TOL = 0.000000001

    # Backtracking line search constant. Slop rate, the fraction of the decrease in V(x)
    # by linear extrapolation that's accepted by the algorithm. Typically on (0.01, 0.3).
    # Constant shared between the j&o vector algorithms
    # 
    # Determined experimentally to be 0.1
    BLS_ALPHA = 0.1

    # Backtracking line search constant. Dampening const, fraction by which the step size (t)
    # is reduced per search iteration: t = beta*t. Typically on (0.1, 0.8) where 0.1 is very
    # crude (aggressive update for t) and 0.8 is less crude (keeps 80% of current t value).
    # Constant shared between the j&o vector algorithms
    # 
    # Determined experimentally to be 0.15
    BLS_BETA = 0.15

    # Gradient Descent constant. The weight relationship between gyro & accel residuals. OG = 50
    # Constant shared between the j&o vector algorithms.
    # 
    # Determined from literature to be 50
    w0 = 50

    # Gradient Descent constant. Weight of gyroscope residuals, follows: wGYRO = sqrt(w0).
    # Don't edit directly! Constant shared between the j&o vector algorithms
    wGYRO = math.sqrt(w0)

    # Gradient Descent constant. Weight of accelerometer residuals, follows: wACCEE = 1/sqrt(w0).
    # Don't edit directly! Constant shared between the j&o vector algorithms
    wACCEL = 1 / math.sqrt(w0)

    # Constant earth gravity in m/s/s, used for zeroing all dof in motion detection
    GRAVITY = 9.807

    # Multiplication factor to convert from radians to degrees
    RAD_TO_DEG = 180 / math.pi

    # Switch to toggle testing which runs the algorithm endlessly until max number of 
    # tests are reached
    RUN_TESTS = False

    # Number of tests to run if RUN_TESTS is true
    TESTS_TO_RUN = 10

    # Ground truth results for the MPU6050 Gimbal based on 10 trials of
    # 5000 samples each.
    # 
    # @todo redo these with the "majority sign tracking" implemented
    GROUND_TRUTH_MPU6050_GIMBAL = {
        "J1": [0.296018916, 0.242142017, 0.923692622],
        "J2": [0.149099431, 0.172561861, 0.972894569],
        "O1": [0.313682761, -0.945354297, 0.075673158],
        "O2": [0.481507229, 0.864035385, -0.144495404],
    }
