import math
import serial
from anatomical_hinge_nagillimi.anatomical_hinge import AnatomicalHinge
from anatomical_hinge_nagillimi.data import Data
from anatomical_hinge_nagillimi.result.joint_result import HingeJointResult
import matplotlib.pyplot as plt
import matplotlib.animation as animation

max_raw_range = 16384 * 2
g_per_raw_range = 2 / max_raw_range
dps_per_raw_range = 250 / max_raw_range

hinge = AnatomicalHinge()
ser = serial.Serial("COM5", 38400)

print("\nConnected to port: ", ser.portstr)
print("Incoming data format")
serialData = ser.readline().split(b"\x09")
print("Length of packet:", len(serialData))
print("Packet type:", type(serialData), type(serialData[0]))
print("First timestamp:", int(serialData[0]), "\n")

# Define the read data function
def readData():
    byteData = ser.readline().split(b"\x09")
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
    return hinge.update(Data(int(ts), a1, g1, a2, g2))

# Calibrate
byteData = ser.readline().split(b"\x09")
while len(byteData) != 0:
    readData()
    if hinge.status == HingeJointResult.STREAMING: break
    
# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

x_len = 100
xs = list(range(0, x_len))
ys = [0] * x_len
ax.set_ylim(-30, 30)

line, = ax.plot(xs, ys)

# Format plot
plt.xticks(rotation=45, ha='right')
plt.subplots_adjust(bottom=0.30)

def animate(i, ys):
    data = readData()
    # print(data)
    ys.append(data)
    ys = ys[-x_len:]
    line.set_ydata(ys)
    return line,

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(ys,), interval=500, blit=True)
plt.show()