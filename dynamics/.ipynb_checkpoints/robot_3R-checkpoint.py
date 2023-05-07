import numpy as np

"""Dynamics of 3R"""
def dynamics_3R(dt, u, th, th_dot, robot_pars):
    #define parameters
    # link lengths

    # link lengths
    a1 = robot_pars['a1']
    a2 = robot_pars['a2']
    a3 = robot_pars['a3']

    #link masses
    m1 = robot_pars['m1']
    m2 = robot_pars['m2']
    m3 = robot_pars['m3']

    # link inertias
    I1 = robot_pars['I1']
    I2 = robot_pars['I2']
    I3 = robot_pars['I3']

    #position of CM
    l1 = robot_pars['l1']
    l2 = robot_pars['l2']
    l3 = robot_pars['l3']

    #dynamic friction coefficient
    fd = robot_pars['fd']

    # gravity
    g = 9.81*0
    # matrix initializations
    
    ##
    #define thetas and theta_dots for convenience
    th1 = th[0]
    th2 = th[1]
    th3 = th[2]
    th1_dot = th_dot[0]
    th2_dot = th_dot[1]
    th3_dot = th_dot[2]

    
    tau = u.reshape(-1,1)

    # mass matrix
    m11 = I1+I2+I3+m1*l1**2+m2*(a1**2+l2**2+2*a1*l2*np.cos(th2))+m3*(a1**2+a2**2+l3**2+2*a1*a2*np.cos(th2)+2*a1*l3*np.cos(th2+th3)+2*a2*l3*np.cos(th3))
    m22 = I2+I3+m2*l2**2+m3*(a2**2+l3**2+2*a2*l3*np.cos(th3))
    m33 = I3+m3*l3**2
    m12 = I2+I3+m2*(l2**2+a1*l2*np.cos(th2))+m3*(a2**2+l3**2+a1*a2*np.cos(th2)+a1*l3*np.cos(th2+th3)+2*a2*l3*np.cos(th3))
    m21 = m12
    m13 = I3+m3*(l3**2+a1*l3*np.cos(th2+th3)+a2*l3*np.cos(th3))
    m31 = m13
    m23 = I3+m3*(l3**2+a2*l3*np.cos(th3))
    m32 = m23


    M = np.array([[m11,m12,m13],[m21,m22,m23],[m31,m32,m33]])
              

    # coriolis terms
    c11 = -(m3*a1*l3*np.sin(th2+th3)+m3*a2*l3*np.sin(th3))*th3_dot-(m2+m3*a1*a2*np.sin(th2)+a1*l2*np.sin(th2)+m3*a1*l3*np.sin(th2+th3))*th2_dot
    c22 = -m3*a2*l3*np.sin(th3)*th3_dot
    c33 = -(m3*a1*a2*np.sin(th2))*th2_dot-(m3*a2*l3*np.sin(th3))*th3_dot
    c12 = -(m2*a1*l2*np.sin(th2)+m3*a1*l3*np.sin(th2+th3)+m3*a1*a2*np.sin(th2))*th1_dot-(m3*a1*a2*np.sin(th2)+m3*a1*l3*np.sin(th2+th3)+m2*a1*l2*np.sin(th2))*th2_dot-(m3*a2*l3*np.sin(th3)+m3*a1*l3*np.sin(th2+th3))*th3_dot
    c13 = -(m3*a1*l3*np.sin(th2+th3)+m3*a2*l3*np.sin(th3))*th1_dot-(m3*a1*l3*np.sin(th2+th3)+m3*a2*l3*np.sin(th3))*th2_dot-(m3*a2*l3*np.sin(th2)+m3*a1*l3*np.sin(th2+th3))*th3_dot
    c21 = (m2*a1*l2*np.sin(th2)+m3*a1*a2*np.sin(th2)+m3*a1*l3*np.sin(th2+th3))*th1_dot-(m3*a2*l3*np.sin(th3))*th3_dot
    c23 = -(m3*a2*l3*np.sin(th3))*th1_dot-(m3*a2*l3*np.sin(th3))*th2_dot-(m3*a2*l3*np.sin(th3))*th3_dot
    c31 = (m3*a1*l3*np.sin(th2+th3)+m3*a2*l3*np.sin(th3))*th1_dot+(m3*a2*l3*np.sin(th3))*th2_dot
    c32 = (m3*a2*l3*np.sin(th3))*th1_dot+(m3*a2*l3*np.sin(th3))*th2_dot-(m3*a2*l3*np.sin(th3))*th3_dot

    C = np.array([[c11,c12,c13],[c21,c22,c23],[c31,c32,c33]])


    # gravity terms
    g1 = (m2*l2+m3*a2)*g*np.cos(th1+th2)+m3*l3*g*np.cos(th1+th2+th3)+(m1*l1+m3*a1+m2*a1)*g*np.cos(th1)
    g2 = (m2*l2+m3*a2)*g*np.cos(th1+th2)+m3*l3*g*np.cos(th1+th2+th3)
    g3 = m3*l3*g*np.cos(th1+th2+th3)


    G = np.array([[g1],[g2],[g3]])
    

    
    # theta evolution
    th_ddot = np.matmul(np.linalg.inv(M),(tau-G-np.matmul(C,th_dot).reshape(-1,1)-(fd*th_dot).reshape(-1,1)).reshape(-1,))
    th_dot_next = th_dot + th_ddot*dt
    th_next = th + th_dot*dt + (1/2)*th_ddot*dt**2

    return th_next, th_dot_next


"""Data Generation for training"""

def dynamics_3R_data_gen(dt,num_traj,num_snaps,num_states,num_inputs, robot_pars):
    
    # link lengths
    a1 = robot_pars['a1']
    a2 = robot_pars['a2']
    a3 = robot_pars['a3']

    # matrix initializations
    X = np.empty((num_traj,num_snaps+1,num_states)) # cartesian state matrix
    U = np.empty((num_traj,num_snaps,num_inputs)) # input matrix
    x_end = np.empty((num_traj,num_snaps+1,2)) # position of end effector

    for i in range(num_traj):
    # initialize the values for the trajectory
      # joint angles
        X[i,0,0:3] = np.pi*(2*np.random.rand(1,1,3)-1) # theta values
          # joint velocities
        X[i,0,3:6] = 0.1*(2*np.random.rand(1,1,3)-1) # theta values
        x_end[i,0,0]= a1*np.cos(X[i,0,0])+a2*np.cos(X[i,0,0]+X[i,0,1])+a3*np.cos(X[i,0,0]+X[i,0,1]+X[i,0,2])
        x_end[i,0,1]= a1*np.sin(X[i,0,0])+a2*np.sin(X[i,0,0]+X[i,0,1])+a3*np.sin(X[i,0,0]+X[i,0,1]+X[i,0,2])


        for j in range(num_snaps):

            # inputs
            U[i,j,:] = 1*(2*np.random.rand(1,1,3)-1)

            # theta evolution
            X[i,j+1,0:3], X[i,j+1,3:6] = dynamics_3R(dt, U[i,j,:], X[i,j,0:3],  X[i,j,3:6], robot_pars)

            x_end[i,j+1,0]= a1*np.cos(X[i,j+1,0])+a2*np.cos(X[i,j+1,0]+X[i,j+1,1])+a3*np.cos(X[i,j+1,0]+X[i,j+1,1]+X[i,j+1,2])
            x_end[i,j+1,1]= a1*np.sin(X[i,j+1,0])+a2*np.sin(X[i,j+1,0]+X[i,j+1,1])+a3*np.sin(X[i,j+1,0]+X[i,j+1,1]+X[i,j+1,2])

    return x_end,X,U