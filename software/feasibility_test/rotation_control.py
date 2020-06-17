import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import quaternion as qn

def estimate_angles(p_c):
    x_c = p_c[0][0]
    y_c = p_c[1][0]
    z_c = p_c[2][0]

    pitch_c = -np.arctan2(z_c, x_c)
    yaw_c = np.arctan2(y_c, x_c)
    roll_c = np.arctan2(z_c, y_c)

    return pitch_c, yaw_c, roll_c

def compute_R(pitch, yaw, roll):

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

    # R = Rx * Ry * Rz (yaw, roll, pitch)
    #R = np.matmul(Rx, Ry)
    #R = np.matmul(R, Rz)

    # R = Rz * Ry * Rx (roll, pitch, yaw)
    R = np.matmul(Rz, Ry)
    R = np.matmul(R, Rx)

    #R = np.matmul(Ry, Rz)
    #R = np.matmul(R, Rx)

    return R

#r2 = np.cross(q, p)

# a reference 3D point.
p_ref = np.array([[1],
                  [0], 
                  [0]])

# target angles.
pitch_t = 0.0
yaw_t = 0.0
roll_t = 0.0

# rotation angles.
pitch = 0.5
yaw =  0.5
roll = 0.5
R = compute_R(pitch, yaw, roll)

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



# control test.
p_gain = 0.1
max_iteration = 50
# point history
p_hist = np.zeros((max_iteration, 3))

# angle history
a_hist = np.zeros((max_iteration, 3))

p_hist[0,0] = p_c[0,0]
p_hist[0,1] = p_c[1,0]
p_hist[0,2] = p_c[2,0]

a_hist[0,0] = pitch_c
a_hist[0,1] = yaw_c
a_hist[0,2] = roll_c


#qr = qn.angle_to_q(roll_c, pitch_c, yaw_c)
qr = qn.angle_to_q(roll, pitch, yaw)

qa = np.array([0, 1, 0, 0])
qb = qn.qrot(qr, qa)

Rq = qn.q_to_r(qr)

print("original R")
print(R)
print("quaternion to R")
print(Rq)

for i in range(max_iteration):
    d_pitch = p_gain * (pitch_t - pitch_c)
    d_yaw   = p_gain * (yaw_t - yaw_c)
    d_roll  = p_gain * (roll_t - roll_c)

    #pitch_c += d_pitch
    #yaw_c += d_yaw
    #roll_c += d_roll

    # rotate current point by delta angles.
    dR = compute_R(d_pitch, d_yaw, d_roll)    
    p_c = np.matmul(dR, p_c)
    
    pitch_c, yaw_c, roll_c = estimate_angles(p_c)

    p_hist[i,0] = p_c[0,0]
    p_hist[i,1] = p_c[1,0]
    p_hist[i,2] = p_c[2,0]

    a_hist[i,0] = pitch_c
    a_hist[i,1] = yaw_c
    a_hist[i,2] = roll_c


fig = plt.figure(2)
ax = fig.add_subplot(211)
ax.plot(a_hist[:, 0], c='r')
ax.plot(a_hist[:, 1], c='g')
ax.plot(a_hist[:, 2], c='b')
plt.title("angle history")
plt.legend(['pitch', 'yaw', 'roll'])

ax = fig.add_subplot(212)
ax.plot(p_hist[:, 0], c='r')
ax.plot(p_hist[:, 1], c='g')
ax.plot(p_hist[:, 2], c='b')
plt.title("point history")
plt.legend(['x', 'y', 'z'])

plt.show()




    

