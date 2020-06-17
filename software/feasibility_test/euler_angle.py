#
# euler_angle.py
# Euler angle lib.
#

import numpy as np
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def compute_R(roll, pitch, yaw):

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

    # R = Rz * Ry * Rx (yaw, pitch, roll)
    R = np.matmul(Rz, Ry)
    R = np.matmul(R, Rx)

    return R

def solve_angle(pt1, pt2, param0=None):

    # solution: roll, pitch, yaw
    if param0 is not None:
        param_lm = param0.reshape((3,1))
    else:
        param_lm = np.matrix([[0.0], [0.0], [0.0]])
        #param_lm[1,0] = np.pi * 0.5



    x = pt1[0]
    y = pt1[1]
    z = pt1[2]

    pt_target = np.matrix([[pt2[0]], [pt2[1]], [pt2[2]]])

    roll = param_lm[0,0]
    pitch = param_lm[1,0]
    yaw = param_lm[2,0]
    
    # compute f
    f_lm = np.matrix([[1.0*x*np.cos(pitch)*np.cos(yaw) + y*(np.sin(pitch)*np.sin(roll)*np.cos(yaw) - 1.0*np.sin(yaw)*np.cos(roll)) + z*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + 1.0*np.sin(roll)*np.sin(yaw))], 
                      [1.0*x*np.sin(yaw)*np.cos(pitch) + y*(np.sin(pitch)*np.sin(roll)*np.sin(yaw) + 1.0*np.cos(roll)*np.cos(yaw)) + z*(np.sin(pitch)*np.sin(yaw)*np.cos(roll) - 1.0*np.sin(roll)*np.cos(yaw))], 
                      [-1.0*x*np.sin(pitch) + 1.0*y*np.sin(roll)*np.cos(pitch) + 1.0*z*np.cos(pitch)*np.cos(roll)]])

    # compute Jacobian
    Jx = np.matrix([[y*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + 1.0*np.sin(roll)*np.sin(yaw)) + z*(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + 1.0*np.sin(yaw)*np.cos(roll)), -1.0*x*np.sin(pitch)*np.cos(yaw) + y*np.sin(roll)*np.cos(pitch)*np.cos(yaw) + z*np.cos(pitch)*np.cos(roll)*np.cos(yaw), -1.0*x*np.sin(yaw)*np.cos(pitch) + y*(-np.sin(pitch)*np.sin(roll)*np.sin(yaw) - 1.0*np.cos(roll)*np.cos(yaw)) + z*(-np.sin(pitch)*np.sin(yaw)*np.cos(roll) + 1.0*np.sin(roll)*np.cos(yaw))], 
                    [y*(np.sin(pitch)*np.sin(yaw)*np.cos(roll) - 1.0*np.sin(roll)*np.cos(yaw)) + z*(-np.sin(pitch)*np.sin(roll)*np.sin(yaw) - 1.0*np.cos(roll)*np.cos(yaw)), -1.0*x*np.sin(pitch)*np.sin(yaw) + y*np.sin(roll)*np.sin(yaw)*np.cos(pitch) + z*np.sin(yaw)*np.cos(pitch)*np.cos(roll), 1.0*x*np.cos(pitch)*np.cos(yaw) + y*(np.sin(pitch)*np.sin(roll)*np.cos(yaw) - 1.0*np.sin(yaw)*np.cos(roll)) + z*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + 1.0*np.sin(roll)*np.sin(yaw))], 
                    [1.0*y*np.cos(pitch)*np.cos(roll) - 1.0*z*np.sin(roll)*np.cos(pitch), -1.0*x*np.cos(pitch) - 1.0*y*np.sin(pitch)*np.sin(roll) - 1.0*z*np.sin(pitch)*np.cos(roll), 0]])
    Jxt = Jx.transpose()
    
    res_lm = f_lm - pt_target
    sd = np.matmul(Jxt, res_lm)

    n_res_lm = np.linalg.norm(res_lm)

    mu = 1e-4

    for iter in range(10):

        H = np.matmul(Jxt, Jx) + np.identity(3) * mu
        invH = np.linalg.inv(H)

        # compute delta param
        delta_p = -np.matmul(invH, sd)

        param = param_lm + delta_p

        roll = param[0,0]
        pitch = param[1,0]
        yaw = param[2,0]

        # compute f
        f = np.matrix([[1.0*x*np.cos(pitch)*np.cos(yaw) + y*(np.sin(pitch)*np.sin(roll)*np.cos(yaw) - 1.0*np.sin(yaw)*np.cos(roll)) + z*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + 1.0*np.sin(roll)*np.sin(yaw))], 
                       [1.0*x*np.sin(yaw)*np.cos(pitch) + y*(np.sin(pitch)*np.sin(roll)*np.sin(yaw) + 1.0*np.cos(roll)*np.cos(yaw)) + z*(np.sin(pitch)*np.sin(yaw)*np.cos(roll) - 1.0*np.sin(roll)*np.cos(yaw))], 
                       [-1.0*x*np.sin(pitch) + 1.0*y*np.sin(roll)*np.cos(pitch) + 1.0*z*np.cos(pitch)*np.cos(roll)]])

        res = f - pt_target

        n_res = np.linalg.norm(res)

        if n_res < n_res_lm:
            n_res_lm = n_res
            f_lm = f
            res_lm = res

            # compute Jacobian
            Jx = np.matrix([[y*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + 1.0*np.sin(roll)*np.sin(yaw)) + z*(-np.sin(pitch)*np.sin(roll)*np.cos(yaw) + 1.0*np.sin(yaw)*np.cos(roll)), -1.0*x*np.sin(pitch)*np.cos(yaw) + y*np.sin(roll)*np.cos(pitch)*np.cos(yaw) + z*np.cos(pitch)*np.cos(roll)*np.cos(yaw), -1.0*x*np.sin(yaw)*np.cos(pitch) + y*(-np.sin(pitch)*np.sin(roll)*np.sin(yaw) - 1.0*np.cos(roll)*np.cos(yaw)) + z*(-np.sin(pitch)*np.sin(yaw)*np.cos(roll) + 1.0*np.sin(roll)*np.cos(yaw))], 
                            [y*(np.sin(pitch)*np.sin(yaw)*np.cos(roll) - 1.0*np.sin(roll)*np.cos(yaw)) + z*(-np.sin(pitch)*np.sin(roll)*np.sin(yaw) - 1.0*np.cos(roll)*np.cos(yaw)), -1.0*x*np.sin(pitch)*np.sin(yaw) + y*np.sin(roll)*np.sin(yaw)*np.cos(pitch) + z*np.sin(yaw)*np.cos(pitch)*np.cos(roll), 1.0*x*np.cos(pitch)*np.cos(yaw) + y*(np.sin(pitch)*np.sin(roll)*np.cos(yaw) - 1.0*np.sin(yaw)*np.cos(roll)) + z*(np.sin(pitch)*np.cos(roll)*np.cos(yaw) + 1.0*np.sin(roll)*np.sin(yaw))], 
                            [1.0*y*np.cos(pitch)*np.cos(roll) - 1.0*z*np.sin(roll)*np.cos(pitch), -1.0*x*np.cos(pitch) - 1.0*y*np.sin(pitch)*np.sin(roll) - 1.0*z*np.sin(pitch)*np.cos(roll), 0]])

            Jxt = Jx.transpose()
            sd = np.matmul(Jxt, res_lm)

            param_lm = param
            
            mu *= 0.1
        else:
            mu *= 10

        print("{0} - {1} - {2}".format(iter, mu, n_res))

        if n_res_lm < 1e-10:
            break

    roll = param_lm[0,0]
    pitch = param_lm[1,0]
    yaw = param_lm[2,0]

    return roll, pitch, yaw


def symbolic_solve():
    x, y, z = sp.symbols("x y z")
    pitch, yaw, roll = sp.symbols("pitch yaw roll")

    # rotation matrix.
    # rotation with repsect to x-axis (roll)
    Rx = sp.Matrix([[1.0,  0.0,  0.0],
                   [0.0,   sp.cos(roll), -sp.sin(roll)],
                   [0.0,   sp.sin(roll),  sp.cos(roll)]])

    # rotation with respect to y-axis (pitch)
    Ry = sp.Matrix([[sp.cos(pitch),  0.0,  sp.sin(pitch)],
                    [0.0,            1.0,     0.0],
                    [-sp.sin(pitch), 0.0, sp.cos(pitch)]])               

    # rotation with respect to z-axis (yaw)
    Rz = sp.Matrix([[sp.cos(yaw), -sp.sin(yaw), 0.0],
                    [sp.sin(yaw),  sp.cos(yaw), 0.0],
                    [0.0,          0.0,         1.0]])


    pt = sp.Matrix([[x],
                    [y],
                    [z]])

    R = Rz*Ry*Rx

    f = R * pt

    print("f")
    print(f)

    print("Jacobian")
    J_roll = f.diff(roll)
    J_pitch = f.diff(pitch)    
    J_yaw = f.diff(yaw)

    J_angle = J_roll
    J_angle = J_angle.row_join(J_pitch)
    J_angle = J_angle.row_join(J_yaw)

    print(J_angle)
    



if __name__ == "__main__":
    #symbolic_solve()

    pt1 = np.array([1.0, 0.0, 0.0])
    pt2 = np.array([0.0, np.sqrt(1/2), np.sqrt(1/2)])
    

    param0 = np.array([0.0, 0.0, 0.0])
    #param0[1] = np.pi * 0.5

    R = compute_R(param0[0], param0[1], param0[2])
    print(R)
    

    roll, pitch, yaw = solve_angle(pt1, pt2, param0=param0)

    R = compute_R(roll, pitch, yaw)
    pt_reproj = np.matmul(R, pt1.transpose())

    print("roll, pitch, yaw : {0}, {1}, {2}".format(roll * 180 / np.pi, pitch * 180 / np.pi, yaw * 180 / np.pi))

    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')  # 3D Axes

    ax.scatter(pt1[0], pt1[1], pt1[2], c='r', marker='o')
    ax.scatter(pt2[0], pt2[1], pt2[2], c='b', marker='o')
    ax.scatter(pt_reproj[0], pt_reproj[1], pt_reproj[2], c='g', marker='x')
    ax.text(pt_reproj[0], pt_reproj[1], pt_reproj[2], "reprojected")
    
    # Draw axes.
    


    plt.show()

    











