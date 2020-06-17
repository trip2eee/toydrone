import numpy as np
import sympy as sp

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def qadd(q, p):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    p0 = p[0]
    p1 = p[1]
    p2 = p[2]
    p3 = p[3]

    r = [q0 + p0, q1 + p1, q2 + p2, q3 + p3]

    return np.array(r)

def qmul(q, p):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    p0 = p[0]
    p1 = p[1]
    p2 = p[2]
    p3 = p[3]

    r0 = q0*p0 - q1*p1 - q2*p2 - q3*p3  # real
    r1 = q0*p1 + q1*p0 + q2*p3 - q3*p2  # i
    r2 = q0*p2 + q2*p0 - q1*p3 + q3*p1  # j
    r3 = q0*p3 + q3*p0 + q1*p2 - q2*p1  # k

    r = [r0, r1, r2, r3]
    
    return np.array(r)

def qrot(qr, qa):
    
    qb = qmul(qr, qa)
    qb = qmul(qb, qconj(qr))

    return qb

def qconj(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    r = [q0, -q1, -q2, -q3]
    
    return np.array(r)

def qnorm(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    n_sqr = q0*q0 + q1*q1 + q2*q2 + q3*q3
    n = np.sqrt(n_sqr)

    return n

def qnorm_sqr(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    n_sqr = q0*q0 + q1*q1 + q2*q2 + q3*q3

    return n_sqr

def qinv(q):
    n2 = qnorm_sqr(q)
    r = qconj(q) / n2

    return r

def q_to_r(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    R = np.zeros((3, 3))
    """
    R[0, 0] = q0*q0 + q1*q1 - q2*q2 - q3*q3    
    R[0, 1] = 2*(q1*q2 - q0*q3)
    R[0, 2] = 2*(q0*q2 + q1*q3)

    R[1, 0] = 2*(q1*q2 + q0*q3)
    R[1, 1] = q0*q0 - q1*q1 + q2*q2 - q3*q3
    R[1, 1] = 1.0 - 2.0*(q1*q1 + q3*q3)

    R[2, 0] = 2*(q1*q3 - q0*q2)
    R[2, 1] = 2*(q0*q1 + q2*q3)
    R[2, 2] = q0*q0 - q1*q1 - q2*q2 + q3*q3
    """

    R[0, 0] = 1.0 - 2.0*(q2*q2 + q3*q3)
    R[0, 1] = 2*(q1*q2 - q0*q3)
    R[0, 2] = 2*(q0*q2 + q1*q3)

    R[1, 0] = 2*(q1*q2 + q0*q3)
    R[1, 1] = 1.0 - 2.0*(q1*q1 + q3*q3)
    R[1, 2] = 2*(q2*q3 - q0*q1)

    R[2, 0] = 2*(q1*q3 - q0*q2)
    R[2, 1] = 2*(q0*q1 + q2*q3)
    R[2, 2] = 1.0 - 2.0*(q1*q1 + q2*q2)

    return R

def solve_qr(qa, qb):
    qr = np.zeros((4, 1))
    qr[0][0] = 1

    xa = qa[1]
    ya = qa[2]
    za = qa[3]

    xb = qb[1]
    yb = qb[2]
    zb = qb[3]

    a = np.array([[xa], 
                  [ya], 
                  [za]])

    b = np.array([[1],
                  [xb],
                  [yb],
                  [zb]])
    
    for iter in range(5):
        q0 = qr[0][0]
        q1 = qr[1][0]
        q2 = qr[2][0]
        q3 = qr[3][0]

        f = np.matrix([[q0**2 + q1**2 + q2**2 + q3**2], 
                       [xa*(q0**2 + q1**2 - q2**2 - q3**2) + ya*(-2*q0*q3 + 2*q1*q2) + za*(2*q0*q2 + 2*q1*q3)], 
                       [xa*(2*q0*q3 + 2*q1*q2) + ya*(q0**2 - q1**2 + q2**2 - q3**2) + za*(-2.0*q1**2 - 2.0*q3**2 + 1.0)], 
                       [xa*(-2*q0*q2 + 2*q1*q3) + ya*(2*q0*q1 + 2*q2*q3) + za*(q0**2 - q1**2 - q2**2 + q3**2)]])

        Jq = np.matrix([[2*q0, 2*q1, 2*q2, 2*q3], 
                        [2*q0*xa + 2*q2*za - 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za, 2*q0*za + 2*q1*ya - 2*q2*xa, -2*q0*ya + 2*q1*za - 2*q3*xa], 
                        [2*q0*ya + 2*q3*xa, -2*q1*ya - 4.0*q1*za + 2*q2*xa, 2*q1*xa + 2*q2*ya, 2*q0*xa - 2*q3*ya - 4.0*q3*za], 
                        [2*q0*za + 2*q1*ya - 2*q2*xa, 2*q0*ya - 2*q1*za + 2*q3*xa, -2*q0*xa - 2*q2*za + 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za]])
        
        # Hessian
        Jqt = Jq.transpose()
        H = np.matmul(Jqt, Jq) + np.identity(4) * 0.0001
        invH = np.linalg.inv(H)
        
        # residual
        res = f - b

        # norm of residual.
        n_res = np.linalg.norm(res)
        print("{0}: {1}".format(iter, n_res))

        # steepest descent
        sd = np.matmul(Jqt, res)

        delta_qr = -np.matmul(invH, sd)

        qr += delta_qr

        print(delta_qr)

    return qr

def solve_qr_lm(qa, qb, qr0=None):

    if qr0 is not None:
        qr = qr0.reshape((4,1))
    else:
        qr = np.matrix([[1.0], [0.0], [0.0], [0.0]])

    q0 = qr[0, 0]
    q1 = qr[1, 0]
    q2 = qr[2, 0]
    q3 = qr[3, 0]

    xa = qa[1]
    ya = qa[2]
    za = qa[3]

    xb = qb[1]
    yb = qb[2]
    zb = qb[3]

    b = np.array([[1],
                  [xb],
                  [yb],
                  [zb]])
    
    mu = 1e-4
    
    qr_lm = qr
    
    f_lm = np.matrix([[q0**2 + q1**2 + q2**2 + q3**2], 
                      [xa*(q0**2 + q1**2 - q2**2 - q3**2) + ya*(-2*q0*q3 + 2*q1*q2) + za*(2*q0*q2 + 2*q1*q3)], 
                      [xa*(2*q0*q3 + 2*q1*q2) + ya*(q0**2 - q1**2 + q2**2 - q3**2) + za*(-2.0*q0*q1 + 2.0*q2*q3)], 
                      [xa*(-2*q0*q2 + 2*q1*q3) + ya*(2*q0*q1 + 2*q2*q3) + za*(q0**2 - q1**2 - q2**2 + q3**2)]])

    # Jacobian
    Jq = np.matrix([[2*q0, 2*q1, 2*q2, 2*q3], 
                    [2*q0*xa + 2*q2*za - 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za, 2*q0*za + 2*q1*ya - 2*q2*xa, -2*q0*ya + 2*q1*za - 2*q3*xa], 
                    [2*q0*ya - 2.0*q1*za + 2*q3*xa, -2.0*q0*za - 2*q1*ya + 2*q2*xa, 2*q1*xa + 2*q2*ya + 2.0*q3*za, 2*q0*xa + 2.0*q2*za - 2*q3*ya], 
                    [2*q0*za + 2*q1*ya - 2*q2*xa, 2*q0*ya - 2*q1*za + 2*q3*xa, -2*q0*xa - 2*q2*za + 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za]])

    # residual
    res_lm = f_lm - b
    n_res_lm = np.linalg.norm(res_lm)

    # steepest descent
    Jqt = Jq.transpose()
    sd = np.matmul(Jqt, res_lm)
    
    for iter in range(20):
        
        # Hessian        
        H = np.matmul(Jqt, Jq) + np.identity(4) * mu
        invH = np.linalg.inv(H)

        # compute delta qr
        delta_qr = -np.matmul(invH, sd)

        qr = qr_lm + delta_qr

        # compute f
        q0 = qr[0, 0]
        q1 = qr[1, 0]
        q2 = qr[2, 0]
        q3 = qr[3, 0]

        f = np.matrix([[q0**2 + q1**2 + q2**2 + q3**2], 
                       [xa*(q0**2 + q1**2 - q2**2 - q3**2) + ya*(-2*q0*q3 + 2*q1*q2) + za*(2*q0*q2 + 2*q1*q3)], 
                       [xa*(2*q0*q3 + 2*q1*q2) + ya*(q0**2 - q1**2 + q2**2 - q3**2) + za*(-2.0*q0*q1 + 2.0*q2*q3)], 
                       [xa*(-2*q0*q2 + 2*q1*q3) + ya*(2*q0*q1 + 2*q2*q3) + za*(q0**2 - q1**2 - q2**2 + q3**2)]])
        # residual
        res = f - b
        
        # norm of residual.
        n_res = np.linalg.norm(res)

        if n_res < n_res_lm:
            qr_lm = qr
            n_res_lm = n_res    
            res_lm = res        

            f_lm = f

            # update Jacobian.
            Jq = np.matrix([[2*q0, 2*q1, 2*q2, 2*q3], 
                            [2*q0*xa + 2*q2*za - 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za, 2*q0*za + 2*q1*ya - 2*q2*xa, -2*q0*ya + 2*q1*za - 2*q3*xa], 
                            [2*q0*ya - 2.0*q1*za + 2*q3*xa, -2.0*q0*za - 2*q1*ya + 2*q2*xa, 2*q1*xa + 2*q2*ya + 2.0*q3*za, 2*q0*xa + 2.0*q2*za - 2*q3*ya], 
                            [2*q0*za + 2*q1*ya - 2*q2*xa, 2*q0*ya - 2*q1*za + 2*q3*xa, -2*q0*xa - 2*q2*za + 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za]])

            # steepest descent
            Jqt = Jq.transpose()
            sd = np.matmul(Jqt, res_lm)
            
            mu *= 0.1

        else:
            mu *= 10

        print("{0}: {1} : {2}".format(iter, mu, n_res))

        if n_res_lm < 1e-10:
            break

        #print(qr_lm)

    return np.array(qr_lm).flatten()

def symbolic_solve_qr(qa, qb):

    xa, ya, za = sp.symbols("xa ya za")
    A = sp.Matrix([[xa], 
                   [ya], 
                   [za]])

    q0, q1, q2, q3 = sp.symbols("q0 q1 q2 q3")
    """
    r00 = 1.0 - 2.0*(q2*q2 + q3*q3)
    r01 = 2*(q1*q2 - q0*q3)
    r02 = 2*(q0*q2 + q1*q3)

    r10 = 2*(q1*q2 + q0*q3)
    r11 = 1.0 - 2.0*(q1*q1 + q3*q3)
    r12 = 2*(q2*q3 - q0*q1)

    r20 = 2*(q1*q3 - q0*q2)
    r21 = 2*(q0*q1 + q2*q3)
    r22 = 1.0 - 2.0*(q1*q1 + q2*q2)
    """

    r00 = q0*q0 + q1*q1 - q2*q2 - q3*q3    
    r01 = 2*(q1*q2 - q0*q3)
    r02 = 2*(q0*q2 + q1*q3)

    r10 = 2*(q1*q2 + q0*q3)
    r11 = q0*q0 - q1*q1 + q2*q2 - q3*q3
    r12 = 2.0*(q2*q3 - q0*q1)

    r20 = 2*(q1*q3 - q0*q2)
    r21 = 2*(q0*q1 + q2*q3)
    r22 = q0*q0 - q1*q1 - q2*q2 + q3*q3
    
    F = sp.Matrix([[q0*q0 + q1*q1 + q2*q2 + q3*q3],
                   [r00*xa + r01*ya + r02*za],
                   [r10*xa + r11*ya + r12*za],
                   [r20*xa + r21*ya + r22*za]])
    
    c = F.subs([(q0, 0), (q1, 0), (q2, 0), (q3, 0)])
    
    print("F")
    print(F)
    
    #print("F_q0")
    F_q0 = F.diff(q0)
    #print(F_q0)

    #print("F_q1")
    F_q1 = F.diff(q1)
    #print(F_q1)

    #print("F_q2")
    F_q2 = F.diff(q2)
    #print(F_q2)

    #print("F_q3")
    F_q3 = F.diff(q3)
    #print(F_q3)

    Jq = F_q0
    Jq = Jq.row_join(F_q1)
    Jq = Jq.row_join(F_q2)
    Jq = Jq.row_join(F_q3)

    print("Jq")
    print(Jq)

    return F


def q_to_angle(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    roll = np.arctan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2))
    #roll = np.arctan2(1 - 2*(q1*q1 + q2*q2), 2*(q0*q1 + q2*q3))
    pitch = np.arcsin(2.0*(q0*q2 - q3*q1))
    yaw = np.arctan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))
    #yaw = np.arctan2(1 - 2*(q2*q2 + q3*q3), 2*(q0*q3 + q1*q2))

    return roll, pitch, yaw

def angle_to_q(roll, pitch, yaw):

    c_roll  = np.cos(roll * 0.5)
    c_pitch = np.cos(pitch * 0.5)
    c_yaw   = np.cos(yaw * 0.5)

    s_roll  = np.sin(roll * 0.5)
    s_pitch = np.sin(pitch * 0.5)
    s_yaw   = np.sin(yaw * 0.5)

    q0 = c_roll*c_pitch*c_yaw + s_roll*s_pitch*s_yaw
    q1 = s_roll*c_pitch*c_yaw - c_roll*s_pitch*s_yaw
    q2 = c_roll*s_pitch*c_yaw + s_roll*c_pitch*s_yaw
    q3 = c_roll*c_pitch*s_yaw - s_roll*s_pitch*c_yaw

    q = [q0, q1, q2, q3]

    return np.array(q)


if __name__ == "__main__":
    
    #qa = np.array([0, np.sqrt(1/5), np.sqrt(3/5), np.sqrt(1/5)])
    #eps = 1e-6
    #qa = np.array([0, np.sqrt(1 - (eps**2 * 2)), eps, eps])

    #qa = np.array([0, 1, 0, 0])

    #qb = np.array([0, np.sqrt(2/3), np.sqrt(1/3), 0])
    #qb = np.array([0, 0, 1, 0])

    #qr = symbolic_solve_qr(qa, qb)

    qa = np.array([0.0, 1.0, 0.0, 0.0])
    qb = np.array([0.0, 0.0, np.sqrt(1/2), np.sqrt(1/2)])
    

    param0 = np.array([0.0, 0.0, 0.0])
    #param0[1] = np.pi * 0.5

    qr0 = angle_to_q(param0[0], param0[1], param0[2])

    print("solve")
    qr = solve_qr_lm(qa, qb, qr0=qr0)

    print("qr: {0}".format(qr))

    r, p, y = q_to_angle(qr)
    print("roll, pitch, yaw: {0}, {1}, {2}".format(r, p, y))
    print("roll, pitch, yaw: {0}, {1}, {2}".format(r * 180 / np.pi, p * 180 / np.pi, y * 180 / np.pi))

    
    #qr = angle_to_q(r, p, y)
    qb2 = qrot(qr, qa)

    print("qr")
    print(qr)

    print("qb")
    print(qb)

    print("qb2")
    print(qb2)

    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')  # 3D Axes

    ax.scatter(qa[1], qa[2], qa[3], c='r', marker='o')
    ax.scatter(qb[1], qb[2], qb[3], c='b', marker='o')
    ax.scatter(qb2[1], qb2[2], qb2[2], c='g', marker='x')
    ax.text(qb2[1], qb2[2], qb2[2], "reprojected")

    plt.show()


