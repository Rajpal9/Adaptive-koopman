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

def forward_map_3R(th, robot_pars):
    
    # link lengths
    a1 = robot_pars['a1']
    a2 = robot_pars['a2']
    a3 = robot_pars['a3']
    
    x_end = a1*np.cos(th[0])+a2*np.cos(th[0]+th[1])+a3*np.cos(th[0]+th[1]+th[2])
    y_end = a1*np.sin(th[0])+a2*np.sin(th[0]+th[1])+a3*np.sin(th[0]+th[1]+th[2])
    
    return np.array([x_end,y_end])


"""Data Generation for training"""

def dynamics_3R_data_gen(dt,num_traj,num_snaps,num_states,num_inputs, robot_pars):
    
    pars_incor = {}

    # matrix initializations
    X_cor = np.empty((num_traj,num_snaps+1,num_states))
    X_incor = np.empty((num_traj,num_snaps+1,num_states))# cartesian state matrix
    U = np.empty((num_traj,num_snaps,num_inputs)) # input matrix
    x_end_cor = np.empty((num_traj,num_snaps+1,2)) # position of end effector
    x_end_incor = np.empty((num_traj,num_snaps+1,2))
    
    for i in range(num_traj):
    # initialize the values for the trajectory
      # joint angles

        for j in range(num_snaps):

            # inputs
            U[i,j,:] = 1*(2*np.random.rand(1,1,3)-1)
            
            pars_cor, pars_incor = get_robot_params(robot_pars,dt*j)
            
            
            if j == 0:
                X_cor[i,0,0:3] = np.pi*(2*np.random.rand(1,1,3)-1) # theta values
                X_incor[i,0,0:3] = X_cor[i,0,0:3] 
                # joint velocities
                X_cor[i,0,3:6] = 0.1*(2*np.random.rand(1,1,3)-1) # theta values
                X_incor[i,0,3:6] = X_cor[i,0,3:6]  
                x_end_cor[i,0,:] = forward_map_3R(X_cor[i,0,0:3], pars_cor)
                x_end_incor[i,0,:] = forward_map_3R(X_incor[i,0,0:3], pars_incor)

            # theta evolution
            X_cor[i,j+1,0:3], X_cor[i,j+1,3:6] = dynamics_3R(dt, U[i,j,:], X_cor[i,j,0:3],  X_cor[i,j,3:6], pars_cor)
            x_end_cor[i,j+1,:] = forward_map_3R(X_cor[i,j+1,0:3], pars_cor)

            X_incor[i,j+1,0:3], X_incor[i,j+1,3:6] = dynamics_3R(dt, U[i,j,:], X_incor[i,j,0:3],  X_incor[i,j,3:6], pars_incor)
            x_end_incor[i,j+1,:] = forward_map_3R(X_incor[i,j+1,0:3], pars_incor)

        
    return x_end_cor,X_cor, x_end_incor,X_incor,U


def get_robot_params(robot_pars,time):
    pars_incor = {}
    if robot_pars["delay"] == True:
        if robot_pars["delay_time"] > time:
            # link lengths
            del_a1 = 0
            del_a2 = 0
            del_a3 = 0

            #link masses
            del_m1 = 0
            del_m2 = 0
            del_m3 = 0

            # link inertias
            del_I1 = 0
            del_I2 = 0
            del_I3 = 0

            #position of CM
            del_l1 = 0
            del_l2 = 0
            del_l3 = 0

            #dynamic friction coefficient
            del_fd = 0
        else:

            # link lengths
            del_a1 = robot_pars['del_a1']
            del_a2 = robot_pars['del_a2']
            del_a3 = robot_pars['del_a3']

            #link masses
            del_m1 = robot_pars['del_m1']
            del_m2 = robot_pars['del_m2']
            del_m3 = robot_pars['del_m3']

            # link inertias
            del_I1 = robot_pars['del_I1']
            del_I2 = robot_pars['del_I2']
            del_I3 = robot_pars['del_I3']

            #position of CM
            del_l1 = robot_pars['del_l1']
            del_l2 = robot_pars['del_l2']
            del_l3 = robot_pars['del_l3']

            #dynamic friction coefficient
            del_fd = robot_pars['del_fd']
    else:
        # link lengths
        del_a1 = robot_pars['del_a1']
        del_a2 = robot_pars['del_a2']
        del_a3 = robot_pars['del_a3']

        #link masses
        del_m1 = robot_pars['del_m1']
        del_m2 = robot_pars['del_m2']
        del_m3 = robot_pars['del_m3']

        # link inertias
        del_I1 = robot_pars['del_I1']
        del_I2 = robot_pars['del_I2']
        del_I3 = robot_pars['del_I3']

        #position of CM
        del_l1 = robot_pars['del_l1']
        del_l2 = robot_pars['del_l2']
        del_l3 = robot_pars['del_l3']

        #dynamic friction coefficient
        del_fd = robot_pars['del_fd']

        # if unc_type == 'constant':
        #     amp_c = amp_c
        # elif unc_type == 'sinusoidal':
        #     amp_c = amp_c*np.sin(0.25*np.pi*(j*dt))
        # elif unc_type == 'none':
        #     amp_c = 0


  



    pars_cor = robot_pars

        # link lengths
    pars_incor['a1'] = robot_pars['a1'] + del_a1
    pars_incor['a2'] = robot_pars['a2'] + del_a2
    pars_incor['a3'] = robot_pars['a3'] + del_a3

    #link masses
    pars_incor['m1'] = robot_pars['m1'] + del_m1
    pars_incor['m2'] = robot_pars['m2'] + del_m2
    pars_incor['m3'] = robot_pars['m3'] + del_m3

    # link inertias
    pars_incor['I1'] = robot_pars['I1'] + del_I1
    pars_incor['I2'] = robot_pars['I2'] + del_I2
    pars_incor['I3'] = robot_pars['I3'] + del_I3

    #position of CM
    pars_incor['l1'] = robot_pars['l1'] + del_l1
    pars_incor['l2'] = robot_pars['l2'] + del_l2
    pars_incor['l3'] = robot_pars['l3'] + del_l3

    #dynamic friction coefficient
    pars_incor['fd'] = robot_pars['fd'] + del_fd

    return pars_cor, pars_incor