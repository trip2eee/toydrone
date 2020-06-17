import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def estimate_angles(p_c):
    x_c = p_c[0][0]
    y_c = p_c[1][0]
    z_c = p_c[2][0]

    pitch_c = -np.arctan2(z_c, x_c)
    yaw_c = np.arctan2(y_c, x_c)
    roll_c = np.arctan2(z_c, y_c)

    return pitch_c, yaw_c, roll_c



# a reference 3D point.
p_ref = np.array([[1],
                  [0], 
                  [0]])

# target angles.
pitch_t = 0.0
yaw_t = 0.0
roll_t = 0.0

# rotation angles.
pitch = 0.3
yaw =  0.3
roll = 0.3

# rotation matrix.
# rotation with repsect to x-axis (roll)
Rx = np.array([[1.0,  0.0,  0.0],
               [0.0, np.cos(roll), -np.sin(roll)],
               [0.0, np.sin(roll),  np.cos(roll)]])

# rotation with respect to y-axis (pitch)
Ry = np.array([[np.cos(pitch), 0.0,  np.sin(pitch)],
               [0.0,          1.0,  0.0],
               [-np.sin(pitch), 0.0, np.cos(pitch)]])               

# rotation with respect to z-axis (yaw)
Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
               [np.sin(yaw),  np.cos(yaw), 0.0],
               [0.0,          0.0,         1.0]])

R = np.matmul(Rx, Ry)
R = np.matmul(R, Rz)
p_c = np.matmul(R, p_ref)       # current point.


# estimate the current angle.
pitch_c, yaw_c, roll_c = estimate_angles(p_c)

print("pitch: {0} --> {1}".format(pitch, pitch_c))
print("yaw: {0} --> {1}".format(yaw, yaw_c))
print("roll: {0} --> {1}".format(roll, roll_c))


fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')  # 3D Axes

# draw axes
ax.plot((0, 1), (0, 0), (0, 0), c='r')    # x-axis
ax.plot((0, 0), (0, 1), (0, 0), c='g')    # y-axis
ax.plot((0, 0), (0, 0), (0, 1), c='b')    # z-axis

ax.scatter(p_ref[0], p_ref[1], p_ref[2], c='b')
ax.scatter(p_c[0], p_c[1], p_c[2], c='r')

plt.show()


    

