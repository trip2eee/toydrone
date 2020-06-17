import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import quaternion as qn
import euler_angle as ea

def estimate_angles(p_c):
    x_c = p_c[0][0]
    y_c = p_c[1][0]
    z_c = p_c[2][0]

    pitch_c = -np.arctan2(z_c, np.sqrt(x_c**2 + y_c**2))
    yaw_c = np.arctan2(y_c, np.sqrt(x_c**2 + z_c**2))
    roll_c = np.arctan2(y_c, np.sqrt(z_c**2 + x_c**2))

    if roll_c >= np.pi:
        roll_c -= np.pi
    elif roll_c <= -np.pi:
        roll_c += np.piecewise

    return pitch_c, yaw_c, roll_c


#r2 = np.cross(q, p)

# a reference 3D point.
p_ref = np.array([[np.sqrt(1/3)],
                  [np.sqrt(1/3)], 
                  [np.sqrt(1/3)]])


p_ref = np.array([[1.0],
                  [1.0], 
                  [1.0]])                  

# rotation angles.
pitch = 0.5
yaw =  0.0
roll = 0.8

R = ea.compute_R(roll, pitch, yaw)

p_c = np.matmul(R, p_ref)       # current point.

# estimate the current angle.
pitch_c, yaw_c, roll_c = estimate_angles(p_c)


print("pitch: {0} --> {1}".format(pitch, pitch_c))
print("yaw: {0} --> {1}".format(yaw, yaw_c))
print("roll: {0} --> {1}".format(roll, roll_c))


Rc = ea.compute_R(roll_c, pitch_c, yaw_c)
InvRc = np.linalg.inv(Rc)
p_reproj = np.matmul(InvRc, p_c)

diff = p_ref - p_reproj
n_diff = np.linalg.norm(diff)
print("|diff|: {0}".format(n_diff))

# angle estimation using quaternion
print("Quaternion")
qa = np.array([0, p_ref[0][0], p_ref[1][0], p_ref[2][0]])
qb = np.array([0, p_c[0][0], p_c[1][0], p_c[2][0]])
qr = qn.solve_qr_lm(qa, qb)
roll_q, pitch_q, yaw_q = qn.q_to_angle(qr)

print("pitch: {0} --> {1}".format(pitch, pitch_q))
print("yaw: {0} --> {1}".format(yaw, yaw_q))
print("roll: {0} --> {1}".format(roll, roll_q))

inv_qr = qn.qinv(qr)
qa_reproj = qn.qrot(inv_qr, qb)
p_reprojq = qa_reproj[1:]

print("inv_qr: {0}".format(inv_qr))

diffq = np.array(p_ref).flatten() - p_reprojq
n_diffq = np.linalg.norm(diffq)
print("|diffq|: {0}".format(n_diffq))

print("Euler")
roll_e, pitch_e, yaw_e = ea.solve_angle(np.array(p_ref).flatten(), np.array(p_c).flatten())

print("pitch: {0} --> {1}".format(pitch, pitch_e))
print("yaw: {0} --> {1}".format(yaw, yaw_e))
print("roll: {0} --> {1}".format(roll, roll_e))

invRe = np.linalg.inv(ea.compute_R(roll_e, pitch_e, yaw_e))
p_reproje = np.matmul(invRe, p_c)

p_reproje = np.array(p_reproje).flatten()

diffe = np.array(p_ref).flatten() - p_reproje
n_diffe = np.linalg.norm(diffe)
print("|diffe|: {0}".format(n_diffe))



fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')  # 3D Axes

# draw axes
ax.plot((0, 1), (0, 0), (0, 0), c='r')    # x-axis
ax.plot((0, 0), (0, 1), (0, 0), c='g')    # y-axis
ax.plot((0, 0), (0, 0), (0, 1), c='b')    # z-axis

ax.scatter(p_ref[0], p_ref[1], p_ref[2], c='b')
ax.text(p_ref[0][0], p_ref[1][0], p_ref[2][0], "ref")

ax.scatter(p_c[0], p_c[1], p_c[2], c='r')
ax.text(p_c[0][0], p_c[1][0], p_c[2][0], "rotated")

ax.scatter(p_reproj[0], p_reproj[1], p_reproj[2], c='g')
ax.text(p_reproj[0][0], p_reproj[1][0], p_reproj[2][0], "rep")

ax.scatter(p_reprojq[0], p_reprojq[1], p_reprojq[2], c='c')
ax.text(p_reprojq[0], p_reprojq[1], p_reprojq[2], "rep_q")

ax.scatter(p_reproje[0], p_reproje[1], p_reproje[2], c='m')
ax.text(p_reproje[0], p_reproje[1], p_reproje[2], "rep_e")

plt.show()




    

