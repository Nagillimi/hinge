import math
import serial
from anatomical_hinge_nagillimi.anatomical_hinge import AnatomicalHinge
from anatomical_hinge_nagillimi.data import Data
from anatomical_hinge_nagillimi.result.joint_result import HingeJointResult

max_raw_range = 16384 * 2
g_per_raw_range = 2 / max_raw_range
dps_per_raw_range = 250 / max_raw_range

hinge = AnatomicalHinge()
ser = serial.Serial("COM5", 38400)

print("\nConnected to port: ", ser.portstr)
print("Incoming data format")
print(ser.readline())
serialData = ser.readline().split(b"\x09")
print("Length of packet:", len(serialData))
print("Packet type:", type(serialData), type(serialData[0]))
print("First timestamp:", int(serialData[0]), "\n")

for x in range(10000):
    byteData = ser.readline().split(b"\x09")
    # print("raw byte array =", byteData)
    ts = int(byteData[0]) / 1000
    a1 = [
        float(byteData[1]) * g_per_raw_range,
        float(byteData[2]) * g_per_raw_range,
        float(byteData[3]) * g_per_raw_range
    ]
    g1 = [
        math.radians(float(byteData[4]) * dps_per_raw_range),
        math.radians(float(byteData[5]) * dps_per_raw_range),
        math.radians(float(byteData[6]) * dps_per_raw_range)
    ]
    a2 = [
        float(byteData[7]) * g_per_raw_range,
        float(byteData[8]) * g_per_raw_range,
        float(byteData[9]) * g_per_raw_range
    ]
    g2 = [
        math.radians(float(byteData[10]) * dps_per_raw_range),
        math.radians(float(byteData[11]) * dps_per_raw_range),
        math.radians(float(byteData[12]) * dps_per_raw_range)
    ]
    # print("a1 =", a1, "(g)")
    # print("g1 =", g1, "(rps)")
    # print("a2 =", a2, "(g)")
    # print("g2 =", g2, "(rps)")
    combinedAngle = hinge.update(Data(int(ts), a1, g1, a2, g2))
    if hinge.status == HingeJointResult.STREAMING:
        print("\n")
        print(x)
        print("Accel angle", hinge.hingeJoint.accelAngle)
        print("Gyro angle", hinge.hingeJoint.gyroAngle.current)
        print("Combined angle", combinedAngle)
