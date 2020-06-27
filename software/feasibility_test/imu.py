# IMU simulation

import numpy as np
import matplotlib.pyplot as plt
import quaternion as qt

if __name__ == "__main__":
    dim_measurements = 3

    # create simulation data.
    seq_len = 150
    T = 1
    gt_angle = np.zeros((seq_len, 3))   # angle GT (roll, pitch, yaw)
    gt_w = np.zeros((seq_len, 3))   # angular velocity GT (roll, pitch, yaw)    

    #gt_w[20:40,:] = [0.00, 0.05, 0.0]
    #gt_w[70:90,:] = [0.00, -0.10, 0.0]

    #gt_w[20:40,:] = [0.05,  0.0,  0.0]
    #gt_w[70:90,:] = [-0.10, 0.0,  0.0]

    #gt_w[20:40,:] = [0.0, 0.0,  0.05]
    #gt_w[70:90,:] = [0.0, 0.0, -0.10]

    gt_w[20:40,:] = [ 0.00,  0.05, 0.0]
    gt_w[50:60,:] = [ 0.01,  -0.05, 0.01]
    gt_w[70:90,:] = [-0.01, -0.00, 0.0]


    for i in range(2, seq_len):
        gt_angle[i] = gt_angle[i-1] + gt_w[i]

    # generate accelerometer G.T.
    gn = [0, 0, 0, 1]
    gt_a = np.zeros((seq_len, 3))     # gx, gy, gz

    # ya,t = -Rt gn + e_a,t
    for i in range(seq_len):
        qr = qt.angle_to_q(gt_angle[i,0], gt_angle[i,1], gt_angle[i,2])
    
        qa = qt.qrot(qr, gn)
        gt_a[i, 0] = -qa[1]
        gt_a[i, 1] = -qa[2]
        gt_a[i, 2] = -qa[3]

    # generate magnetometer G.T.
    gt_m = np.zeros((seq_len, 3))
    mn = np.array([0, 1, 0, 0])

    for i in range(seq_len):
        qr = qt.angle_to_q(gt_angle[i,0], gt_angle[i,1], gt_angle[i,2])

        qm = qt.qrot(qr, mn)
        b = np.zeros((3, 1))
        b[0,0] = qm[1]
        b[1,0] = qm[2]
        b[2,0] = qm[3]

        a0 = gt_a[i,0]
        a1 = gt_a[i,1]
        a2 = gt_a[i,2]

        A = np.zeros((3,3))
        A[0,0] = a1*a1 + a2*a2
        A[0,1] = -a0*a1
        A[0,2] = -a0*a2

        A[1,0] = -a0*a1
        A[1,1] = a0*a0 + a2*a2
        A[1,2] = -a1*a2

        A[2,0] = -a0*a2
        A[2,1] = -a1*a2
        A[2,2] = a0*a0 + a1*a1

        if np.linalg.matrix_rank(A) < 3:
            gt_m[i, 0] = 1
            gt_m[i, 1] = 0
            gt_m[i, 2] = 0
        else:
            m = np.linalg.solve(A, b)

            gt_m[i, 0] = m[0,0]
            gt_m[i, 1] = m[1,0]
            gt_m[i, 2] = m[2,0]

    
    # generate noisy measurements
    mu_w = 0.0
    mu_a = 0.0
    mu_m = 0.0

    sigma_w = 0.00001
    sigma_a = 0.00001
    sigma_m = 0.001

    y_w  = np.zeros((seq_len, 3))       # gyroscope measurement
    y_a = np.zeros((seq_len, 3))        # accelerometer measurement
    y_m = np.zeros((seq_len, 3))

    for i in range(seq_len):
        y_w[i,:] = gt_w[i,:] + np.random.normal(mu_w, sigma_w, 3)
        y_a[i,:] = gt_a[i,:] + np.random.normal(mu_a, sigma_a, 3)
        y_m[i,:] = gt_m[i,:] + np.random.normal(mu_m, sigma_m, 3)

    # Kalman filter state history
    hist_a  = np.zeros((seq_len, 3))      # states.
    hist_ap = np.zeros((seq_len, 3))      # predicted states.

    # for each time t
    x = np.matrix([[1],
                   [0],
                   [0],
                   [0]])

    xp = x

    P = np.eye(4) * 1.0
    Pp = P

    Q = np.eye(3)*sigma_w*sigma_w
    
    R = np.eye(dim_measurements)
    R[0,0] = sigma_a*sigma_a
    R[1,1] = sigma_a*sigma_a
    R[2,2] = sigma_a*sigma_a

    if dim_measurements == 6:
        R[3,3] = sigma_m*sigma_m
        R[4,4] = sigma_m*sigma_m
        R[5,5] = sigma_m*sigma_m

    for t in range(2, seq_len):
        
        # angular velocity measurements.
        w0 = y_w[t, 0]
        w1 = y_w[t, 1]
        w2 = y_w[t, 2]

        # quaternion states.
        q0 = x[0, 0]
        q1 = x[1, 0]
        q2 = x[2, 0]
        q3 = x[3, 0]

        # prediction
        xp = np.zeros((4, 1))
        xp[0, 0] = -((T*w0*q1) / 2.0) - ((T*w1*q2) / 2.0) - ((T*w2*q3) / 2.0) + q0
        xp[1, 0] =  ((T*w0*q0) / 2.0) - ((T*w1*q3) / 2.0) + ((T*w2*q2) / 2.0) + q1
        xp[2, 0] =  ((T*w0*q3) / 2.0) + ((T*w1*q0) / 2.0) - ((T*w2*q1) / 2.0) + q2
        xp[3, 0] = -((T*w0*q2) / 2.0) + ((T*w1*q1) / 2.0) + ((T*w2*q0) / 2.0) + q3

        F = np.zeros((4, 4))
        F[0, 0] =  1.0
        F[0, 1] = -T*w0/2
        F[0, 2] = -T*w1/2
        F[0, 3] = -T*w2/2

        F[1, 0] =  T*w0/2
        F[1, 1] =  1.0
        F[1, 2] =  T*w2/2
        F[1, 3] = -T*w1/2

        F[2, 0] =  T*w1/2
        F[2, 1] = -T*w2/2
        F[2, 2] =  1.0
        F[2, 3] =  T*w0/2

        F[3, 0] =  T*w2/2
        F[3, 1] =  T*w1/2
        F[3, 2] = -T*w0/2
        F[3, 3] =  1.0
        
        G = np.zeros((4, 3))
        G[0, 0] =  T*q1/2
        G[0, 1] =  T*q2/2
        G[0, 2] =  T*q3/2

        G[1, 0] = -T*q0/2
        G[1, 1] =  T*q3/2
        G[1, 2] = -T*q2/2

        G[2, 0] = -T*q3/2
        G[2, 1] = -T*q0/2
        G[2, 2] =  T*q1/2

        G[3, 0] =  T*q2/2
        G[3, 1] = -T*q1/2
        G[3, 2] = -T*q0/2

        Pp = np.matmul(F, np.matmul(P, F.transpose())) + np.matmul(G, np.matmul(Q, G.transpose()))

        # Update
        q0 = xp[0, 0]
        q1 = xp[1, 0]
        q2 = xp[2, 0]
        q3 = xp[3, 0]

        Rt = np.zeros((3, 3))
        Rt[0, 0] = 2*q0*q0 + 2*q1*q1 - 1.0
        Rt[0, 1] = -2*q0*q3 + 2*q1*q2
        Rt[0, 2] = 2*q0*q2 + 2*q1*q3

        Rt[1, 0] = 2*q0*q3 + 2*q1*q2
        Rt[1, 1] = 2*q0*q0 + 2*q2*q2 - 1
        Rt[1, 2] = -2*q0*q1 + 2*q2*q3

        Rt[2, 0] = -2*q0*q2 + 2*q1*q3
        Rt[2, 1] = 2*q0*q1 + 2*q2*q3
        Rt[2, 2] = 2*q0*q0 + 2*q3*q3 - 1

        # measurement matrix.
        H = np.zeros((dim_measurements, 4))
        H[0, 0] = -2*q2
        H[0, 1] = -2*q3
        H[0, 2] = -2*q0
        H[0, 3] = -2*q1

        H[1, 0] =  2*q1
        H[1, 1] =  2*q0
        H[1, 2] = -2*q3
        H[1, 3] = -2*q2

        H[2, 0] = -4*q0
        H[2, 1] =  0
        H[2, 2] =  0
        H[2, 3] = -4*q3

        gn = np.matrix([[0], [0], [1]])
        py_a = -Rt*gn
        ea = np.array(y_a[t,:]).flatten() - np.array(py_a).flatten()
        e = np.array([ea[0], ea[1], ea[2]])

        if dim_measurements == 6:
            H[3, 0] = 4*q0
            H[3, 1] = 4*q1
            H[3, 2] = 0
            H[3, 3] = 0
            
            H[4, 0] = 2*q3
            H[4, 1] = 2*q2
            H[4, 2] = 2*q1
            H[4, 3] = 2*q0

            H[5, 0] = -2*q2
            H[5, 1] =  2*q3
            H[5, 2] = -2*q0
            H[5, 3] =  2*q1

        
            mn = np.matrix([[1], [0], [0]])
            py_m = Rt*mn
        
            em = np.array(y_m[t,:]).flatten() - np.array(py_m).flatten()       # TODO: To simulate magnetometer
            e = np.array([ea[0], ea[1], ea[2], em[0], em[1], em[2]])

        e = e.reshape((dim_measurements, 1))

        S = np.matmul(np.matmul(H, Pp), H.transpose()) + R
        K = np.matmul(np.matmul(Pp, H.transpose()), np.linalg.inv(S))

        x = xp + np.matmul(K, e)
        P = Pp - np.matmul(np.matmul(K, S), K.transpose())


        norm_x = np.linalg.norm(x)
        Jt = np.matmul(x, x.transpose())/(norm_x*norm_x*norm_x)

        x = x / norm_x
        P = np.matmul(np.matmul(Jt, P), Jt.transpose()) 

        norm_xp = np.linalg.norm(xp)
        xp = xp / norm_xp
                

        # compute angles.
        """
        gn = np.array([0, 0, 0, 1])
        qr = qt.solve_qr(gn, xp.flatten())
        roll, pitch, yaw = qt.q_to_angle(qr)

        hist_ap[t, :] = [roll, pitch, yaw]

        qr = qt.solve_qr(gn, x.flatten())
        roll, pitch, yaw = qt.q_to_angle(qr)

        hist_a[t, :] = [roll, pitch, yaw]
        """

        rollp, pitchp, yawp = qt.q_to_angle(np.array(xp).flatten())
        hist_ap[t, :] = [rollp, pitchp, yawp]

        roll, pitch, yaw = qt.q_to_angle(np.array(x).flatten())
        hist_a[t, :] = [roll, pitch, yaw]

    plt.figure("Angles")
    plt.subplot(311)
    plt.plot(gt_angle[:,0], c='g')
    plt.plot(hist_ap[:,0], c='c')
    plt.plot(hist_a[:,0], c='b')
    plt.legend(['ground truth', 'prediction', 'update'])
    plt.title("roll")

    plt.subplot(312)
    plt.plot(gt_angle[:,1], c='g')
    plt.plot(hist_ap[:,1], c='c')
    plt.plot(hist_a[:,1], c='b')
    plt.legend(['ground truth', 'prediction', 'update'])
    plt.title("pitch")

    plt.subplot(313)
    plt.plot(gt_angle[:,2], c='g')
    plt.plot(hist_ap[:,2], c='c')
    plt.plot(hist_a[:,2], c='b')
    plt.legend(['ground truth', 'prediction', 'update'])
    plt.title("yaw")

    plt.figure("Angular velocities")
    plt.subplot(311)
    plt.plot(gt_w[:,0], c='g')
    plt.plot(y_w[:,0],  c='m')
    plt.title("roll rate")

    plt.subplot(312)
    plt.plot(gt_w[:,1], c='g')
    plt.plot(y_w[:,1],  c='m')
    plt.title("pitch rate")

    plt.subplot(313)
    plt.plot(gt_w[:,2], c='g')
    plt.plot(y_w[:,2],  c='m')
    plt.title("yaw rate")

    plt.figure("Acceleration")
    plt.subplot(311)
    plt.plot(gt_a[:,0], c='g')
    plt.plot(y_a[:,0], c='m')
    plt.title("gx")

    plt.subplot(312)
    plt.plot(gt_a[:,1], c='g')
    plt.plot(y_a[:,1], c='m')
    plt.title("gy")

    plt.subplot(313)
    plt.plot(gt_a[:,2], c='g')
    plt.plot(y_a[:,2], c='m')
    plt.title("gz")


    
    plt.show()


