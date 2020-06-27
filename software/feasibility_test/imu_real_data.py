# IMU simulation
# Test with the real data obtained by Arduino nano 33 BLE.
#
# Accelerometer
# - unit: G
# - Sample rate: 119Hz
# 
# Gyroscope
# - unit: deg/sec
# - Sample rate: 119Hz
# 
# Magnetometer
# -unit: uT
# -Sample rate: 20Hz
#
# Test data format
# - <timestamp> -> ax ay az wx wy wz mx my mz T(us)

import numpy as np
import matplotlib.pyplot as plt
import quaternion as qt

def compute_q(y):

    weight_n = 100
    weight_a = 100
    weight_m = 1
    
    y_a = y[0:3]
    y_w = y[3:6] * np.pi / 180.0     # deg/s -> rad/s
    y_m = y[6:9]
    T = y[9] * 1e-6                 # us -> s

    # normalize.
    y_a = y_a / np.linalg.norm(y_a)
    y_m = y_m / np.linalg.norm(y_m)

    qr = np.matrix([[1.0], [0.0], [0.0], [0.0]])

    q0 = qr[0, 0]
    q1 = qr[1, 0]
    q2 = qr[2, 0]
    q3 = qr[3, 0]

    xa = y_a[0]
    ya = y_a[1]
    za = y_a[2]
    
    mb = np.cross(y_a, np.cross(y_m, y_a))
    mb0 = mb[0]
    mb1 = mb[1]
    mb2 = mb[2]

    b = np.array([[1 * weight_n],
                  
                  [0],
                  [0],
                  [1 * weight_a],

                  [1],
                  [0],
                  [0]])
    
    mu = 1e-4
    
    qr_lm = qr
    
    f_lm = np.matrix([[q0**2 + q1**2 + q2**2 + q3**2], 

                      [xa*(q0**2 + q1**2 - q2**2 - q3**2) + ya*(-2*q0*q3 + 2*q1*q2) + za*(2*q0*q2 + 2*q1*q3)], 
                      [xa*(2*q0*q3 + 2*q1*q2) + ya*(q0**2 - q1**2 + q2**2 - q3**2) + za*(-2.0*q0*q1 + 2.0*q2*q3)], 
                      [xa*(-2*q0*q2 + 2*q1*q3) + ya*(2*q0*q1 + 2*q2*q3) + za*(q0**2 - q1**2 - q2**2 + q3**2)],
                      
                      [mb0*(q0**2 + q1**2 - q2**2 - q3**2) - 2*mb1*(q0*q3 - q1*q2) + 2*mb2*(q0*q2 + q1*q3)], 
                      [2*mb0*(q0*q3 + q1*q2) + mb1*(q0**2 - q1**2 + q2**2 - q3**2) - 2.0*mb2*(q0*q1 - q2*q3)], 
                      [-2*mb0*(q0*q2 - q1*q3) + 2*mb1*(q0*q1 + q2*q3) + mb2*(q0**2 - q1**2 - q2**2 + q3**2)]])
    f_lm[0,0] *= weight_n
    f_lm[1:3,0] *= weight_a

    # Jacobian
    Jq = np.matrix([[2*q0, 2*q1, 2*q2, 2*q3], 

                    [2*q0*xa + 2*q2*za - 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za, 2*q0*za + 2*q1*ya - 2*q2*xa, -2*q0*ya + 2*q1*za - 2*q3*xa], 
                    [2*q0*ya - 2.0*q1*za + 2*q3*xa, -2.0*q0*za - 2*q1*ya + 2*q2*xa, 2*q1*xa + 2*q2*ya + 2.0*q3*za, 2*q0*xa + 2.0*q2*za - 2*q3*ya], 
                    [2*q0*za + 2*q1*ya - 2*q2*xa, 2*q0*ya - 2*q1*za + 2*q3*xa, -2*q0*xa - 2*q2*za + 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za],
                    
                    [2*mb0*q0 - 2*mb1*q3 + 2*mb2*q2, 2*mb0*q1 + 2*mb1*q2 + 2*mb2*q3, -2*mb0*q2 + 2*mb1*q1 + 2*mb2*q0, -2*mb0*q3 - 2*mb1*q0 + 2*mb2*q1],
                    [2*mb0*q3 + 2*mb1*q0 - 2.0*mb2*q1, 2*mb0*q2 - 2*mb1*q1 - 2.0*mb2*q0, 2*mb0*q1 + 2*mb1*q2 + 2.0*mb2*q3, 2*mb0*q0 - 2*mb1*q3 + 2.0*mb2*q2],
                    [-2*mb0*q2 + 2*mb1*q1 + 2*mb2*q0, 2*mb0*q3 + 2*mb1*q0 - 2*mb2*q1, -2*mb0*q0 + 2*mb1*q3 - 2*mb2*q2, 2*mb0*q1 + 2*mb1*q2 + 2*mb2*q3]])
    Jq[0,:] *= weight_n
    Jq[1:3,:] *= weight_a

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
                       [xa*(-2*q0*q2 + 2*q1*q3) + ya*(2*q0*q1 + 2*q2*q3) + za*(q0**2 - q1**2 - q2**2 + q3**2)],
                          
                       [mb0*(q0**2 + q1**2 - q2**2 - q3**2) - 2*mb1*(q0*q3 - q1*q2) + 2*mb2*(q0*q2 + q1*q3)], 
                       [2*mb0*(q0*q3 + q1*q2) + mb1*(q0**2 - q1**2 + q2**2 - q3**2) - 2.0*mb2*(q0*q1 - q2*q3)], 
                       [-2*mb0*(q0*q2 - q1*q3) + 2*mb1*(q0*q1 + q2*q3) + mb2*(q0**2 - q1**2 - q2**2 + q3**2)]])
        f[0,0] *= weight_n
        f[1:3,0] *= weight_a

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
                            [2*q0*za + 2*q1*ya - 2*q2*xa, 2*q0*ya - 2*q1*za + 2*q3*xa, -2*q0*xa - 2*q2*za + 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za],
                            
                            [2*mb0*q0 - 2*mb1*q3 + 2*mb2*q2, 2*mb0*q1 + 2*mb1*q2 + 2*mb2*q3, -2*mb0*q2 + 2*mb1*q1 + 2*mb2*q0, -2*mb0*q3 - 2*mb1*q0 + 2*mb2*q1],
                            [2*mb0*q3 + 2*mb1*q0 - 2.0*mb2*q1, 2*mb0*q2 - 2*mb1*q1 - 2.0*mb2*q0, 2*mb0*q1 + 2*mb1*q2 + 2.0*mb2*q3, 2*mb0*q0 - 2*mb1*q3 + 2.0*mb2*q2],
                            [-2*mb0*q2 + 2*mb1*q1 + 2*mb2*q0, 2*mb0*q3 + 2*mb1*q0 - 2*mb2*q1, -2*mb0*q0 + 2*mb1*q3 - 2*mb2*q2, 2*mb0*q1 + 2*mb1*q2 + 2*mb2*q3]])
            Jq[0,:] *= weight_n
            Jq[1:3,:] *= weight_a

            # steepest descent
            Jqt = Jq.transpose()
            sd = np.matmul(Jqt, res_lm)
            
            mu *= 0.1

        else:
            mu *= 10

        #print("{0}: {1} : {2}".format(iter, mu, n_res))

        if n_res_lm < 1e-10:
            break

    return np.array(qr_lm).flatten()

if __name__ == "__main__":
    dim_measurements = 6

    sensor_data = []
    data_file = "IMU_raw2.txt"
    # load data.
    with open(data_file, "r") as f:
        while True:
            l_data = f.readline()
            if l_data == "":
                break
            l_data = l_data.split()[2:]
            for l in l_data:
                sensor_data.append(float(l))

    sensor_data = np.reshape(sensor_data, (-1, 10))
    
    # create simulation data.
    seq_len = sensor_data.shape[0]

    # Kalman filter state history
    hist_a  = np.zeros((seq_len, 3))      # states.
    hist_ap = np.zeros((seq_len, 3))      # predicted states.
    hist_solve = np.zeros((seq_len, 3))      # predicted states.

        
    qr = compute_q(sensor_data[0,:])
    qr = qt.qconj(qr)
    roll, pitch, yaw = qt.q_to_angle(qr)
    hist_solve[0,:] = [roll, pitch, yaw]

    x = np.matrix([[qr[0]],
                   [qr[1]],
                   [qr[2]],
                   [qr[3]]])    

    xp = x

    P = np.eye(4) * (0.1 * 0.1)
    Pp = P
    
    mu_w = 0.0
    mu_a = 0.0
    mu_m = 0.0

    sigma_a = 0.01      # accelerometer
    sigma_w = 0.5 * np.pi / 180.0       # gyroscope
    sigma_m = 100.0       # magnetometer

    Q = np.eye(3)*sigma_w*sigma_w
    
    R = np.eye(dim_measurements)
    R[0,0] = sigma_a*sigma_a
    R[1,1] = sigma_a*sigma_a
    R[2,2] = sigma_a*sigma_a

    if dim_measurements == 6:
        R[3,3] = sigma_m*sigma_m
        R[4,4] = sigma_m*sigma_m
        R[5,5] = sigma_m*sigma_m

    for t in range(1, seq_len):
        
        y_a = sensor_data[t,0:3]
        y_w = sensor_data[t,3:6] * np.pi / 180.0
        y_m = sensor_data[t,6:9]
        T = sensor_data[t,9] * 1e-6
        
        y_a = y_a / np.linalg.norm(y_a)
        y_m = y_m / np.linalg.norm(y_m)
        
        #y_w[0] = 0
        #y_w[1] = 0
        #y_w[2] = 0

        # angular velocity measurements.
        w0 = y_w[0]
        w1 = y_w[1]
        w2 = y_w[2]

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
        ea = np.array(y_a).flatten() - np.array(py_a).flatten()
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

            #y_mc = np.cross(y_a, np.cross(y_m, y_a))

            em = np.array(y_m).flatten() - np.array(py_m).flatten()       # TODO: To simulate magnetometer
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

        qr = compute_q(sensor_data[t,:])
        qr = qt.qconj(qr)
        roll, pitch, yaw = qt.q_to_angle(qr)
        hist_solve[t,:] = [roll, pitch, yaw]

    plt.figure("Angles")
    plt.subplot(311)
    #plt.plot(gt_angle[:,0], c='g')
    plt.plot(hist_ap[:,0], c='c')
    plt.plot(hist_a[:,0], c='b')
    plt.plot(hist_solve[:,0], c='r')
    plt.legend(['prediction', 'update', 'optimization'])
    plt.title("roll")

    plt.subplot(312)
    #plt.plot(gt_angle[:,1], c='g')
    plt.plot(hist_ap[:,1], c='c')
    plt.plot(hist_a[:,1], c='b')
    plt.plot(hist_solve[:,1], c='r')
    plt.legend(['prediction', 'update', 'optimization'])
    plt.title("pitch")

    plt.subplot(313)
    #plt.plot(gt_angle[:,2], c='g')
    plt.plot(hist_ap[:,2], c='c')
    plt.plot(hist_a[:,2], c='b')
    plt.plot(hist_solve[:,2], c='r')
    plt.legend(['prediction', 'update', 'optimization'])
    plt.title("yaw")


    plt.figure("Angular velocities")
    plt.subplot(311)
    #plt.plot(gt_w[:,0], c='g')
    plt.plot(sensor_data[:,3],  c='m')
    plt.title("roll rate")

    plt.subplot(312)
    #plt.plot(gt_w[:,1], c='g')
    plt.plot(sensor_data[:,4],  c='m')
    plt.title("pitch rate")

    plt.subplot(313)
    #plt.plot(gt_w[:,2], c='g')
    plt.plot(sensor_data[:,5],  c='m')
    plt.title("yaw rate")
    

    plt.figure("Acceleration")
    plt.subplot(311)
    #plt.plot(gt_a[:,0], c='g')
    plt.plot(sensor_data[:,0],  c='m')
    plt.title("gx")

    plt.subplot(312)
    #plt.plot(gt_a[:,1], c='g')
    plt.plot(sensor_data[:,1],  c='m')
    plt.title("gy")

    plt.subplot(313)
    #plt.plot(gt_a[:,2], c='g')
    plt.plot(sensor_data[:,2],  c='m')
    plt.title("gz")

    plt.figure("Magnetometer")
    plt.subplot(311)
    #plt.plot(gt_a[:,0], c='g')
    plt.plot(sensor_data[:,6],  c='m')
    plt.title("mx")

    plt.subplot(312)
    #plt.plot(gt_a[:,1], c='g')
    plt.plot(sensor_data[:,7],  c='m')
    plt.title("my")

    plt.subplot(313)
    #plt.plot(gt_a[:,2], c='g')
    plt.plot(sensor_data[:,8],  c='m')
    plt.title("mz")
    

    
    plt.show()


