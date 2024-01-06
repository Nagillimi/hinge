# Anatomical Hinge Joint Algorithm Library

## Background

This python library provides realtime estimations for a hinge angle for a saddle or spherical joint. Designed specifically for humans and knee joints, the alorithm works best in reporting flexion/extension; or any angle component that dominates the range of motion during calibration. This implementation is based off of the research done by Thomas Seel, Jörg Raisch, and Thomas Schauer [1] and includes some of the updated process from [2].

Currently, this library exists on the *TestPyPi* index and plans to be upgraded to *PyPi* eventually.

For learning reasons, all linear algebra, gradient desscent, backtracking line search, and sum of sqaures optimization was computed by hand and will therefore not be that performant- but it is lightweight. In the future, a `numpy` branch will be created to handle all these methods internally to use less boilerplate.

## API

### Data

A `Data` object collects the incoming data into a readable package and is included in the export of this library:

```python
data = Data(ts: int, a1: list[float], g1:  list[float], a2:  list[float], g2: list[float])
```

### Update

To update the system and collect the estimated hinge angle, call:

```python
hinge = AnatomicalHinge()
angle = hinge.update(data)
```

> Note, if the system is anything other than the `RUN` state, `angle` will be 0

### State

The library functions as a state machine, following:

```python
class State(Enum):
    # analyzes incoming data to provide an appropriate downsampling index for the algorithms
    DETECT_DOWNSAMPLING_INDEX = 1
    # joint axis and pose estimations
    CALIBRATING = 2
    # hinge angle streaming, all algorithms are finished
    RUN = 3
    # used to initialize all states and restart
    RESET = 4
```

Each state follows the other automatically and contains specific status messages. You may modify state directly if you'd like to opt out of say, the claibration or downsampling detection phases. Only the `RUN` phase will output/stream the hinge angle (as a `float`).

### Status

To check the status of the algorithm, call `hinge.status` which reports a union state based on the current state:

```python
status = Union[CalibrationResult, HingeJointResult, KinematicResult, TemporalResult]

class CalibrationResult(Enum):
    SUCCESS = "Calibration succeeded"
    WAITING_FOR_MOTION = "Waiting for motion"
    INITIALIZED = "Initial conditions set"
    NOT_ENOUGH_UNIQUE_MOTION = "Not enough unique motion data yet"
    MOTION_ERROR_ABOVE_THRESHOLD = "Sum of squares error of motion above threshold"

class HingeJointResult(Enum):
    SETTING_INITIAL_CONDITIONS = "Setting ICs from average buffer"
    INITIAL_CONDITIONS_SET = "ICs from average buffer are set"
    STREAMING = "Streaming hinge angle"

class KinematicResult(Enum):
    MOTION_DETECTED = "Significant motion detected"
    CONSECUTIVE_MOTION_DETECTED = "Consecutive significant motion detected"
    MOTION_UNDER_THRESHOLD = "Insignificant motion detected"
    STILLNESS_DETECTED = "Stillness detected"
    CONSECUTIVE_STILLNESS_DETECTED = "Significant stillness detected"

class TemporalResult(Enum):
    OPTIMAL = "Current signal sampling is optimal for calibration"
    ABOVE = "Downsampled signal sampling since it's above optimal values"
    BELOW = "No downsampling required, signal sampling is below minimum threshold. Uncharted territory."
    NOT_ENOUGH_SAMPLES = "Not enough signal data yet"
```

## Example

Using two common MPU6050's, an example jupyter notebook exists [here](https://github.com/Nagillimi/hinge/blob/master/examples/serial_test.ipynb) which takes in inertial data over a serial com port and runs the library in real-time (with some debugging results).

An example that updates a graph using `matplotlib` is currently in progress.

## Process

Simply put, the system detects an ideal downsampling index based on the incoming timestamps, calibrates the joint axis vectors and offsets (wrt to the inertial sensors), and computes the hinge angle based on those joint axis constants. Only datasets with significant enough motion will be included in the calculation to avoid local minima; this process is done in the `Kinematics` class.

Overall, the hinge algorithm assumes:

1. the placement of the inertial sensors will not change at any point while running
2. the placement of the inertial sensors brackets the joint in question
3. the range of motion of the component in interest is larger than the others during calibration (not typically a problem if looking at flexion/extension on human joints)

### Calibration

As per the algorithm process laid out in [1] & [2], the 2 joint axis estimations are computed first for each inertial sensor. By definition, this the estimated hinge joint vector represented by the calibration motion data wrt that sensor. Using this vector, the joint offsets are computed afterwards using the same motion data. These are by definition the positions (in *mm*) to the hinge joint centre wrt each sensor. Each joint vector is represented in spherical coordinates to cut down on the number of unknowns, since there are reaslistically only 2dof given the kinematic constraints.

If you're interested, the calibration output is collected into a `SolutionSet` object, shown here with all its children:

```python
class SolutionSet:
    # the converged accumulation of sum of squares error (cost)
    sumOfSquares: float

    # the final derivative of the sum of squares error (dcost)
    derivSumOfSquares: float

    # the converged solution set
    x: XSphere

class XSphere:
    def __init__(
            self,
            t1 = random.random(),
            p1 = random.random(),
            t2 = random.random(),
            p2 = random.random()
    ):
        self.vector1 = SphericalCoordinate(t1, p1)
        self.vector2 = SphericalCoordinate(t2, p2)

class SphericalCoordinate:
    def __init__(self, theta: float, phi: float):
        self.theta = theta
        self.phi = phi

    def toRectangular(self):
        return self.normalize([
            math.cos(self.theta) * math.cos(self.phi),
            math.cos(self.phi) * math.sin(self.theta),
            math.sin(self.phi)
        ])
```

### Development with a Ground Truth

Pure hinge joint created with mecano, includes a potentiometer as a ground truth hinge angle. The controller sends inertial data from these 2 MPU6050's along with timestamps and works well with the example code for handling serial data in Python.

![ ](/pics/IMG_0733.JPG)

## References

[1] Seel, T.; Raisch, J.; Schauer, T. IMU-Based Joint Angle Measurement for Gait Analysis. Sensors 2014, 14, 6891-6909. https://doi.org/10.3390/s140406891 
[2] Olsson, F.; Kok, M.; Seel, T.; Halvorsen, K. Robust Plug-and-Play Joint Axis Estimation Using Inertial Sensors. Sensors 2020, 20, 3534. https://doi.org/10.3390/s20123534 