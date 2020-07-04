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

    weight_n = 1
    weight_a = 1
    weight_m = 1
    
    y_a = y[0:3]
    y_m = y[6:9]

    # normalize.
    y_a = y_a / np.linalg.norm(y_a)
    y_m = y_m / np.linalg.norm(y_m)

    y_m = np.cross(y_a, np.cross(y_m, y_a))
    y_m = y_m / np.linalg.norm(y_m)
    
    qr = np.matrix([[1.0], [0.0], [0.0], [0.0]])

    q0 = qr[0, 0]
    q1 = qr[1, 0]
    q2 = qr[2, 0]
    q3 = qr[3, 0]

    xa = y_a[0]
    ya = y_a[1]
    za = y_a[2]
    
    xm = y_m[0]
    ym = y_m[1]
    zm = y_m[2]

    b = np.array([[1 * weight_n],
                  
                  [0],
                  [0],
                  [-1 * weight_a],

                  [1 * weight_m],
                  [0],
                  [0]])
    
    mu = 1e-4
    
    qr_lm = qr
    
    f_lm = np.matrix([[q0**2 + q1**2 + q2**2 + q3**2], 

                      [xa*(q0**2 + q1**2 - q2**2 - q3**2) + ya*(-2*q0*q3 + 2*q1*q2) + za*(2*q0*q2 + 2*q1*q3)], 
                      [xa*(2*q0*q3 + 2*q1*q2) + ya*(q0**2 - q1**2 + q2**2 - q3**2) + za*(-2.0*q0*q1 + 2.0*q2*q3)], 
                      [xa*(-2*q0*q2 + 2*q1*q3) + ya*(2*q0*q1 + 2*q2*q3) + za*(q0**2 - q1**2 - q2**2 + q3**2)],
                      
                      [xm*(q0**2 + q1**2 - q2**2 - q3**2) + ym*(-2*q0*q3 + 2*q1*q2) + zm*(2*q0*q2 + 2*q1*q3)], 
                      [xm*(2*q0*q3 + 2*q1*q2) + ym*(q0**2 - q1**2 + q2**2 - q3**2) + zm*(-2.0*q0*q1 + 2.0*q2*q3)], 
                      [xm*(-2*q0*q2 + 2*q1*q3) + ym*(2*q0*q1 + 2*q2*q3) + zm*(q0**2 - q1**2 - q2**2 + q3**2)]])

    f_lm[0,0] *= weight_n
    f_lm[1:4,0] *= weight_a
    f_lm[4:7,0] *= weight_m

    # Jacobian
    Jq = np.matrix([[2*q0, 2*q1, 2*q2, 2*q3], 

                    [2*q0*xa + 2*q2*za - 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za, 2*q0*za + 2*q1*ya - 2*q2*xa, -2*q0*ya + 2*q1*za - 2*q3*xa], 
                    [2*q0*ya - 2.0*q1*za + 2*q3*xa, -2.0*q0*za - 2*q1*ya + 2*q2*xa, 2*q1*xa + 2*q2*ya + 2.0*q3*za, 2*q0*xa + 2.0*q2*za - 2*q3*ya], 
                    [2*q0*za + 2*q1*ya - 2*q2*xa, 2*q0*ya - 2*q1*za + 2*q3*xa, -2*q0*xa - 2*q2*za + 2*q3*ya, 2*q1*xa + 2*q2*ya + 2*q3*za],
                    
                    [2*q0*xm + 2*q2*zm - 2*q3*ym, 2*q1*xm + 2*q2*ym + 2*q3*zm, 2*q0*zm + 2*q1*ym - 2*q2*xm, -2*q0*ym + 2*q1*zm - 2*q3*xm], 
                    [2*q0*ym - 2.0*q1*zm + 2*q3*xm, -2.0*q0*zm - 2*q1*ym + 2*q2*xm, 2*q1*xm + 2*q2*ym + 2.0*q3*zm, 2*q0*xm + 2.0*q2*zm - 2*q3*ym], 
                    [2*q0*zm + 2*q1*ym - 2*q2*xm, 2*q0*ym - 2*q1*zm + 2*q3*xm, -2*q0*xm - 2*q2*zm + 2*q3*ym, 2*q1*xm + 2*q2*ym + 2*q3*zm]])

    Jq[0,:] *= weight_n
    Jq[1:4,:] *= weight_a
    Jq[4:7,:] *= weight_m

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
                          
                       [xm*(q0**2 + q1**2 - q2**2 - q3**2) + ym*(-2*q0*q3 + 2*q1*q2) + zm*(2*q0*q2 + 2*q1*q3)], 
                       [xm*(2*q0*q3 + 2*q1*q2) + ym*(q0**2 - q1**2 + q2**2 - q3**2) + zm*(-2.0*q0*q1 + 2.0*q2*q3)], 
                       [xm*(-2*q0*q2 + 2*q1*q3) + ym*(2*q0*q1 + 2*q2*q3) + zm*(q0**2 - q1**2 - q2**2 + q3**2)]])

        f[0,0] *= weight_n
        f[1:4,0] *= weight_a
        f[4:7,0] *= weight_m

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
                            
                            [2*q0*xm + 2*q2*zm - 2*q3*ym, 2*q1*xm + 2*q2*ym + 2*q3*zm, 2*q0*zm + 2*q1*ym - 2*q2*xm, -2*q0*ym + 2*q1*zm - 2*q3*xm], 
                            [2*q0*ym - 2.0*q1*zm + 2*q3*xm, -2.0*q0*zm - 2*q1*ym + 2*q2*xm, 2*q1*xm + 2*q2*ym + 2.0*q3*zm, 2*q0*xm + 2.0*q2*zm - 2*q3*ym], 
                            [2*q0*zm + 2*q1*ym - 2*q2*xm, 2*q0*ym - 2*q1*zm + 2*q3*xm, -2*q0*xm - 2*q2*zm + 2*q3*ym, 2*q1*xm + 2*q2*ym + 2*q3*zm]])

            Jq[0,:] *= weight_n
            Jq[1:4,:] *= weight_a
            Jq[4:6,:] *= weight_m

            # steepest descent
            Jqt = Jq.transpose()
            sd = np.matmul(Jqt, res_lm)
            
            mu *= 0.1

        else:
            mu *= 10

        #print("{0}: {1} : {2}".format(iter, mu, n_res))

        if n_res_lm < 1e-10:
            break
    
    result = np.array(qr_lm).flatten()
    result = result / np.linalg.norm(result)

    return result

if __name__ == "__main__":
    dim_measurements = 4

    sensor_data = []
    data_file = "IMU_raw2.txt"
    # load data.
    with open(data_file, "r") as f:
        while True:
            l_data = f.readline()
            if l_data == "":
                break
            l_data = l_data.split()[2:]

            if len(l_data) != 10:
                print("Error")
                
            for l in l_data:
                sensor_data.append(float(l))

    sensor_data = np.reshape(sensor_data, (-1, 10))

    # coordinate change.
    sensor_data[:,2] *= -1.0        # negate z_a

    sensor_data[:,5] *= -1.0        # negate z_w

    sensor_data[:,6] *= -1.0        # negate x_m
    sensor_data[:,8] *= -1.0        # negate z_m
    
    # create simulation data.
    seq_len = sensor_data.shape[0]

    # Kalman filter state history
    hist_angle  = np.zeros((seq_len, 3))          # states.
    hist_angle_pred = np.zeros((seq_len, 3))      # predicted states.
    
    hist_q_z  = np.zeros((seq_len, 4))         # quaternion measurements
    hist_angle_z = np.zeros((seq_len, 3))      # angle of quaternion measurements.
    
    hist_bias_w = np.zeros((seq_len, 3))        # gyroscope bias history
    hist_q = np.zeros((seq_len, 4))             # quaternion
    hist_q_pred = np.zeros((seq_len, 4))        # predicted quaternion

    qr = compute_q(sensor_data[0,:])
    qr = qr / np.linalg.norm(qr)

    roll, pitch, yaw = qt.q_to_angle(qr)
    hist_angle_z[0,:] = [roll, pitch, yaw]

    y_w = sensor_data[0,3:6] * np.pi / 180.0

    x = np.matrix([[qr[0]],
                   [qr[1]],
                   [qr[2]],
                   [qr[3]],
                   [y_w[0]],
                   [y_w[1]],
                   [y_w[2]]]).astype(np.float32)

    xp = x

    hist_angle[0,:] = [roll, pitch, yaw]
    hist_angle_pred[0,:] = [roll, pitch, yaw]
    
    hist_q_z[0,:] = np.array(x[0:4]).flatten()
    hist_q[0,:] = np.array(x[0:4]).flatten()
    hist_q_pred[0,:] = np.array(x[0:4]).flatten()
    hist_bias_w[0,:] = np.array(x[4:7]).flatten()
    
    
    sigma_w = 0.3      # gyroscope
    sigma_bw = 0.01
    sigma_q = 0.1
    sigma_q_r = 0.15

    
    P = np.eye(7).astype(np.float32) * (0.1 * 0.1)
    P[4,4] = sigma_bw * sigma_bw
    P[5,5] = sigma_bw * sigma_bw
    P[6,6] = sigma_bw * sigma_bw
            
    Qf = np.zeros((7, 7)).astype(np.float32)
    Qf[0,0] = sigma_q * sigma_q
    Qf[1,1] = sigma_q * sigma_q
    Qf[2,2] = sigma_q * sigma_q
    Qf[3,3] = sigma_q * sigma_q

    Qf[4,4] = sigma_bw * sigma_bw
    Qf[5,5] = sigma_bw * sigma_bw
    Qf[6,6] = sigma_bw * sigma_bw

    Qe = np.zeros((3,3)).astype(np.float32)
    Qe[0,0] = sigma_w * sigma_w
    Qe[1,1] = sigma_w * sigma_w
    Qe[2,2] = sigma_w * sigma_w

        
    R = np.eye(dim_measurements).astype(np.float32) * sigma_q_r * sigma_q_r

    f = open("imu_test.txt", "w")

    t=0
    y_a = sensor_data[t,0:3]
    y_w = sensor_data[t,3:6] * np.pi / 180.0
    y_m = sensor_data[t,6:9]    
    T = sensor_data[t,9] * 1e-6
    
    f.write("{0} {1} {2} ".format(y_a[0], y_a[1], y_a[2]))
    f.write("{0} {1} {2} ".format(y_w[0], y_w[1], y_w[2]))
    f.write("{0} {1} {2} ".format(y_m[0], y_m[1], y_m[2]))
    f.write("{0} ".format(T))

    sensor_data[0,6:9] = sensor_data[0,6:9] / np.linalg.norm(sensor_data[0,6:9])
    sensor_data[t,6:9] = np.cross(y_a, np.cross(sensor_data[t,6:9], y_a))

    for v in np.array(x).flatten():
        f.write("{0} ".format(v))
    
    for v in np.array(P).flatten():
        f.write("{0} ".format(v))

    f.write("\n")
    
    for t in range(1, seq_len):
        
        y_a = sensor_data[t,0:3]
        y_w = sensor_data[t,3:6] * np.pi / 180.0
        y_m = sensor_data[t,6:9]
        T = sensor_data[t,9] * 1e-6
        
        y_a = y_a.astype(np.float32)
        y_w = y_w.astype(np.float32)
        y_m = y_m.astype(np.float32)
        T = T.astype(np.float32)

        # angular velocity measurements.
        w0 = y_w[0]
        w1 = y_w[1]
        w2 = y_w[2]

        # quaternion states.
        q0 = x[0, 0]
        q1 = x[1, 0]
        q2 = x[2, 0]
        q3 = x[3, 0]

        bw0 = x[4, 0]
        bw1 = x[5, 0]
        bw2 = x[6, 0]

        # prediction
        xp = np.zeros((7, 1)).astype(np.float32)
        xp[0, 0] = -T*q1*(-bw0 + w0)/2 - T*q2*(-bw1 + w1)/2 - T*q3*(-bw2 + w2)/2 + q0 
        xp[1, 0] =  T*q0*(-bw0 + w0)/2 + T*q2*(-bw2 + w2)/2 - T*q3*(-bw1 + w1)/2 + q1 
        xp[2, 0] =  T*q0*(-bw1 + w1)/2 - T*q1*(-bw2 + w2)/2 + T*q3*(-bw0 + w0)/2 + q2 
        xp[3, 0] =  T*q0*(-bw2 + w2)/2 + T*q1*(-bw1 + w1)/2 - T*q2*(-bw0 + w0)/2 + q3
        xp[4, 0] = bw0
        xp[5, 0] = bw1
        xp[6, 0] = bw2

        F = np.eye(7).astype(np.float32)
        F[0, 0] =  1.0
        F[0, 1] = -T*(w0 - bw0)/2
        F[0, 2] = -T*(w1 - bw1)/2
        F[0, 3] = -T*(w2 - bw2)/2

        F[0, 4] =  T*q1/2
        F[0, 5] =  T*q2/2
        F[0, 6] =  T*q3/2

        F[1, 0] =  T*(w0 - bw0)/2
        F[1, 1] =  1.0
        F[1, 2] =  T*(w2 - bw2)/2
        F[1, 3] = -T*(w1 - bw1)/2

        F[1, 4] = -T*q0/2
        F[1, 5] =  T*q3/2
        F[1, 6] = -T*q2/2

        F[2, 0] =  T*(w1 - bw1)/2
        F[2, 1] = -T*(w2 - bw2)/2
        F[2, 2] =  1.0
        F[2, 3] =  T*(w0 - bw0)/2

        F[2, 4] = -T*q3/2
        F[2, 5] = -T*q0/2
        F[2, 6] =  T*q1/2


        F[3, 0] =  T*(w2 - bw2)/2
        F[3, 1] =  T*(w1 - bw1)/2
        F[3, 2] = -T*(w0 - bw0)/2
        F[3, 3] =  1.0
        
        F[3, 4] =  T*q2/2
        F[3, 5] = -T*q1/2
        F[3, 6] = -T*q0/2
        
        #print(F)

        G = np.zeros((7, 3)).astype(np.float32)
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

        Pp = np.matmul(F, np.matmul(P, F.transpose())) + Qf + np.matmul(G, np.matmul(Qe, G.transpose()))

        # Update
        # measurement.
        qz = compute_q(sensor_data[t,:])

        q_cur = np.array(qz).flatten()
        if np.linalg.norm(xp[0:4,0]-q_cur) > np.linalg.norm(xp[0:4,0]+q_cur):
            qz = -qz


        # measurement matrix. I
        H = np.zeros((4, 7)).astype(np.float32)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        H[3, 3] = 1.0
        
        #print(H)

        res = qz.reshape((4,1)).astype(np.float32) - np.matmul(H, xp)
        
        S = np.matmul(np.matmul(H, Pp), H.transpose()) + R
        invS = np.linalg.inv(S).astype(np.float32)

        K = np.matmul(np.matmul(Pp, H.transpose()), invS).astype(np.float32)

        x = xp + np.matmul(K, res)
        P = Pp - np.matmul(np.matmul(K, H), Pp)
    
        #x = xp

        x4 = x[0:4,:]
        norm_x = np.linalg.norm(x4)
        Jt4 = np.matmul(x4, x4.transpose())/(norm_x*norm_x*norm_x)
        
        Jt = np.zeros((7, 7)).astype(np.float32)
        Jt[0:4, 0:4] = Jt4
        Jt[4,4] = 1.0
        Jt[5,5] = 1.0
        Jt[6,6] = 1.0

        x[0:4] = x[0:4] / norm_x
        P = np.matmul(np.matmul(Jt, P), Jt.transpose()) 
        
        x = x.astype(np.float32)
        P = P.astype(np.float32)

        f.write("{0} {1} {2} ".format(y_a[0], y_a[1], y_a[2]))
        f.write("{0} {1} {2} ".format(y_w[0], y_w[1], y_w[2]))
        f.write("{0} {1} {2} ".format(y_m[0], y_m[1], y_m[2]))
        f.write("{0} ".format(T))

        for v in np.array(x).flatten():
            f.write("{0} ".format(v))
        
        for v in np.array(P).flatten():
            f.write("{0} ".format(v))

        f.write("\n")

        norm_xp = np.linalg.norm(xp[0:4])
        xp[0:4] = xp[0:4] / norm_xp

        rollp, pitchp, yawp = qt.q_to_angle(np.array(xp).flatten())
        if yawp < 0.0:
            yawp = yawp + (np.pi * 2)

        hist_angle_pred[t, :] = [rollp, pitchp, yawp]

        roll, pitch, yaw = qt.q_to_angle(np.array(x).flatten())
        
        if yaw < 0.0:
            yaw = yaw + (np.pi * 2)

        hist_angle[t, :] = [roll, pitch, yaw]

        # measurement.
        qr = compute_q(sensor_data[t,:])
        
        roll, pitch, yaw = qt.q_to_angle(qz)
        if yaw < 0.0:
            yaw = yaw + (np.pi * 2)
            
        hist_angle_z[t,:] = [roll, pitch, yaw]
        
        hist_bias_w[t,:] = x[4:7, 0]
        hist_q[t,:] = x[0:4, 0]
        hist_q_pred[t,:] = xp[0:4, 0]
        hist_q_z[t,:] = qz
    
        sensor_data[t,6:9] = sensor_data[t,6:9] / np.linalg.norm(sensor_data[t,6:9])
        sensor_data[t,6:9] = np.cross(y_a, np.cross(sensor_data[t,6:9], y_a))

    f.close()

    # read the results of C++ implementation.
    idx_frame = 0
    hist_xp_cpp = np.zeros((seq_len, 7))
    hist_x_cpp = np.zeros((seq_len, 7))
    hist_angle_cpp = np.zeros((seq_len, 3))

    with open("imu_result_cpp.txt", "r") as f:

        while True:
            l = f.readline()

            if l != "":
                val = [float(v) for v in l.split()]
                hist_xp_cpp[idx_frame,:] = val[0:7]
                hist_x_cpp[idx_frame,:] = val[7:14]
                hist_angle_cpp[idx_frame, :] = val[14:17]
                if hist_angle_cpp[idx_frame, 2] < 0.0:
                    hist_angle_cpp[idx_frame, 2] += 2*np.pi
                idx_frame += 1
            else:
                break



    plt.figure("Angles")
    plt.subplot(311)
    plt.plot(hist_angle_z[:,0], c='r')
    plt.plot(hist_angle_pred[:,0], c='c')
    plt.plot(hist_angle[:,0], c='b')
    plt.plot(hist_angle_cpp[:,0], c='deepskyblue')
    plt.legend(['measurement', 'prediction', 'update'])
    plt.title("roll")

    plt.subplot(312)
    plt.plot(hist_angle_z[:,1], c='r')
    plt.plot(hist_angle_pred[:,1], c='c')
    plt.plot(hist_angle[:,1], c='b')
    plt.plot(hist_angle_cpp[:,1], c='deepskyblue')
    plt.legend(['measurement', 'prediction', 'update'])
    plt.title("pitch")

    plt.subplot(313)
    plt.plot(hist_angle_z[:,2], c='r')
    plt.plot(hist_angle_pred[:,2], c='c')
    plt.plot(hist_angle[:,2], c='b')
    plt.plot(hist_angle_cpp[:,2], c='deepskyblue')
    plt.legend(['measurement', 'prediction', 'update'])
    plt.title("yaw")


    plt.figure("Angular velocities")
    plt.subplot(311)
    plt.plot(sensor_data[:,3] * np.pi / 180.0,  c='g')
    plt.plot(hist_bias_w[:,0],  c='r')
    plt.plot(hist_x_cpp[:,4], c='m')
    plt.legend(['w_x','bias'])
    plt.title("roll rate")

    plt.subplot(312)
    plt.plot(sensor_data[:,4] * np.pi / 180.0,  c='g')
    plt.plot(hist_bias_w[:,1],  c='r')
    plt.plot(hist_x_cpp[:,5], c='m')
    plt.legend(['w_y','bias'])
    plt.title("pitch rate")

    plt.subplot(313)
    plt.plot(sensor_data[:,5] * np.pi / 180.0,  c='g')
    plt.plot(hist_bias_w[:,2],  c='r')
    plt.plot(hist_x_cpp[:,6], c='m')
    plt.legend(['w_z','bias'])
    plt.title("yaw rate")
    

    plt.figure("Acceleration")
    plt.subplot(311)
    plt.plot(sensor_data[:,0],  c='m')
    plt.title("gx")

    plt.subplot(312)
    plt.plot(sensor_data[:,1],  c='m')    
    plt.title("gy")

    plt.subplot(313)
    plt.plot(sensor_data[:,2],  c='m')    
    plt.title("gz")

    plt.figure("Magnetometer")
    plt.subplot(311)
    plt.plot(sensor_data[:,6],  c='m')
    plt.title("mx")

    plt.subplot(312)
    plt.plot(sensor_data[:,7],  c='m')
    plt.title("my")

    plt.subplot(313)
    plt.plot(sensor_data[:,8],  c='m')
    plt.title("mz")
    
    plt.figure("Quaternion")
    plt.subplot(411)
    plt.plot(hist_q_z[:,0], c='r')
    plt.plot(hist_q_pred[:,0], c='g')
    plt.plot(hist_q[:,0], c='b')
    plt.plot(hist_xp_cpp[:,0], c='lime')
    plt.plot(hist_x_cpp[:,0], c='deepskyblue')

    plt.subplot(412)
    plt.plot(hist_q_z[:,1], c='r')
    plt.plot(hist_q_pred[:,1], c='g')
    plt.plot(hist_q[:,1], c='b')
    plt.plot(hist_xp_cpp[:,1], c='lime')
    plt.plot(hist_x_cpp[:,1], c='deepskyblue')

    plt.subplot(413)
    plt.plot(hist_q_z[:,2], c='r')
    plt.plot(hist_q_pred[:,2], c='g')
    plt.plot(hist_q[:,2], c='b')
    plt.plot(hist_xp_cpp[:,2], c='lime')
    plt.plot(hist_x_cpp[:,2], c='deepskyblue')

    plt.subplot(414)
    plt.plot(hist_q_z[:,3], c='r')
    plt.plot(hist_q_pred[:,3], c='g')
    plt.plot(hist_q[:,3], c='b')
    plt.plot(hist_xp_cpp[:,3], c='lime')
    plt.plot(hist_x_cpp[:,3], c='deepskyblue')

    plt.show()


