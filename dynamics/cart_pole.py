import numpy as np



def cart_pole_dyn(dt, x, F, pars):
    """
    dt = timestep
    x = [x, theta, x_dt, theta_dt];
    F = Force Input
    pars = system parameters
    
    """
    
    m_c = pars["m_c"]    # mass of cart 
    m_p = pars["m_p"]    # mass of pendulum
    l = pars["l"]        # length of pendulum            
    g = 9.81             # acceleration of gravity
    I_p = m_p*(l**2)/12  # MOI of Pendulum
    f_p = pars["f_p"]    # pendulum friction
    f_c = pars["f_c"]    # cart friction
    
    
    ## dynamics of the system
    den = m_p*(l**2)/(4*(m_c+m_p)*(I_p + (m_p*(l**2)/4)))
    x_ddot =(den/(1+den))*g*np.cos(x[1])*np.sin(x[1]) - (m_p*l/(2*(m_c+m_p)))*((x[3]**2)*np.sin(x[1]) +f_p*x[3]*np.cos(x[1]))+  (F-f_c*x[1])/(m_c + m_p)
    th_ddot = m_p*(l/2)*(g*np.sin(x[1])-x_ddot*np.cos(x[1]))/(I_p + m_p*((l/2)**2)) - f_p*x[3]
                                                              
    x_d_next = x_ddot*dt + x[2]
    x_next = (1/2)*x_ddot*(dt**2) + x[2]*dt + x[0]
    
    th_d_next = th_ddot*dt + x[3]
    th_next = (1/2)*th_ddot*(dt**2) + x[3]*dt + x[1]
    
    x_new = np.array([x_next, th_next, x_d_next, th_d_next])
    
    return x_new.reshape(4,)

def cart_pole_fkine(x,pars):
    l = pars["l"]        # length of pendulum 
    
    #end effector position of cartpole
    x_p = x[0] + l*np.sin(x[1])
    y_p = l*np.cos(x[1])
    x_p_dot = l*np.cos(x[1])*x[3]
    y_p_dot = -l*np.sin(x[1])*x[3]
    
    x_end_eff = np.array([x_p,y_p,x_p_dot,y_p_dot])
    
    return x_end_eff

from dynamics.path_generator import cubic_spline_interpolation

def cart_pole_data_gen(dt,num_traj,num_snaps, pars,controller):
    
    """
    A function for generating the data for the cart pole
    dt = time step
    num_traj = number of trajectories
    num_snap = number of snaps in each trajectory
    robot = robot
    controller = random or model partitioning controller (controller)
    """
    num_states = 2
    num_inputs = 1
    
    num_states_cart = 2
    
    X = np.zeros((num_traj, num_snaps, 2*num_states))
    F= np.zeros((num_traj, num_snaps-1, num_inputs))
    X_end = np.zeros((num_traj, num_snaps, 2*num_states_cart))
    
    
    if controller == 'random':
        for i in range(num_traj):
            X[i,0,0] = 2*np.random.rand(1,) - 1
            X[i,0,1] = (np.pi/2)*(2*np.random.rand(1,) - 1)
            X[i,0,2] = 0.1*(2*np.random.rand(1,) - 1)
            X[i,0,3] = 0.1*(2*np.random.rand(1,) - 1)

            #fkine(q)
            X_end[i,0,:] = cart_pole_fkine(X[i,0,:], pars);
            for j in range(num_snaps-1):
                F[i,j,:] = 0.1*(2*np.random.rand(num_inputs,1) - 1).reshape(num_inputs,); #input torques
     
                X[i,j+1,:] = cart_pole_dyn(dt, X[i,j,:], F[i,j,:], pars);
                X_end[i,j+1,:] = cart_pole_fkine(X[i,0,:], pars);

        
    else:
        for i in range(num_traj):
            num_wp = 10
            # generate path
            t_end = dt*num_snaps; 
            t_wp = np.zeros((num_wp,))
            q_wp = np.zeros((num_states, num_wp))
            for s in range(num_wp):
                if s == 0:
                    X[i,0,0] = 2*np.random.rand(1,) - 1
                    X[i,0,1] = (np.pi/2)*(2*np.random.rand(1,) - 1)
                    X[i,0,2] = 0.1*(2*np.random.rand(1,) - 1)
                    X[i,0,3] = 0.1*(2*np.random.rand(1,) - 1)
                    q_wp[:,s] = X[i,0,:2]
                    t_wp[s] = 0;
                    q_wp_dot = 0.07*(2*np.random.rand(num_states,) - 1);
                else:
                    t_wp[s] = s*t_end/num_wp;
                    q_wp[:,s] = q_wp[:,s-1] + q_wp_dot*(t_wp[s]-t_wp[s-1]);
                    q_wp_dot = 0.07*(2*np.random.rand(num_states,) - 1);


            q_traj, qd_traj, qddot_traj = cubic_spline_interpolation(np.transpose(q_wp),t_end,num_snaps-1);

            X[i,0,:num_states] = q_traj[:,0];
            X[i,0,num_states:] = qd_traj[:,0];

            X_end[i,0,:] = cart_pole_fkine(X[i,0,:], pars);

            Kp = 16;
            Kv = 8;
            F_dash = np.zeros((num_states,num_snaps))
            for j in range(num_snaps-1):
                F_dash[:,j] = qddot_traj[:,j] + Kv*(qd_traj[:,j] - X[i,j,num_states:]) + Kp*(q_traj[:,j] - X[i,j,:num_states]);
                F[i,j,:] =  (pars['m_p']+ pars['m_c'])*F_dash[0,j]

                X[i,j+1,:] = cart_pole_dyn(dt, X[i,j,:], F[i,j,:], pars);
                X_end[i,j+1,:] = cart_pole_fkine(X[i,0,:], pars);



    return X_end, X, F
    
    
def cart_pole_data_gen_multi(dt,num_traj,num_snaps, pars, pars_2,controller):
    
    """
    A function for generating the data for the two cart pole systems
    dt = time step
    num_traj = number of trajectories
    num_snap = number of snaps in each trajectory
    robot = 1st cart pole
    robot_2 = 2nd cart pole
    controller = random or model partitioning controller (controller)
    """
    num_states = 2
    num_inputs = 1
    
    num_states_cart = 2
    
    X_1 = np.zeros((num_traj, num_snaps, 2*num_states))
    X_2 = np.zeros((num_traj, num_snaps, 2*num_states))

    X_end_1 = np.zeros((num_traj, num_snaps, 2*num_states_cart))
    X_end_2 = np.zeros((num_traj, num_snaps, 2*num_states_cart))
    
    F = np.zeros((num_traj, num_snaps-1, num_inputs))
    
    
    if controller == 'random':
        for i in range(num_traj):
            X_1[i,0,0] = 2*np.random.rand(1,) - 1
            X_1[i,0,1] = (np.pi/4)*(2*np.random.rand(1,) - 1)
            X_1[i,0,2] = 0.01*(2*np.random.rand(1,) - 1)
            X_1[i,0,3] = 0.1*(2*np.random.rand(1,) - 1)
            
            X_2[i,0,:] = X_1[i,0,:]
            
            #fkine(q)
            X_end_1[i,0,:] = cart_pole_fkine(X_1[i,0,:], pars);
            X_end_2[i,0,:] = cart_pole_fkine(X_2[i,0,:], pars_2);
            for j in range(num_snaps-1):
                F[i,j,:] = 3*(2*np.random.rand(num_inputs,1) - 1).reshape(num_inputs,); #input torques
     
                X_1[i,j+1,:] = cart_pole_dyn(dt, X_1[i,j,:], F[i,j,:], pars);
                X_end_1[i,j+1,:] = cart_pole_fkine(X_1[i,j+1,:], pars);
                
                X_2[i,j+1,:] = cart_pole_dyn(dt, X_2[i,j,:], F[i,j,:], pars_2);
                X_end_2[i,j+1,:] = cart_pole_fkine(X_2[i,j+1,:], pars_2);

        
    else:
        for i in range(num_traj):
            num_wp = 10
            # generate path
            t_end = dt*num_snaps; 
            t_wp = np.zeros((num_wp,))
            q_wp = np.zeros((num_states, num_wp))
            for s in range(num_wp):
                if s == 0:
                    X_1[i,0,0] = 2*np.random.rand(1,) - 1
                    X_1[i,0,1] = (np.pi/2)*(2*np.random.rand(1,) - 1)
                    X_1[i,0,2] = 0.1*(2*np.random.rand(1,) - 1)
                    X_1[i,0,3] = 0.1*(2*np.random.rand(1,) - 1)
                    t_wp[s] = 0;
                    q_wp_dot = 0.07*(2*np.random.rand(num_states,) - 1);
                else:
                    t_wp[s] = s*t_end/num_wp;
                    q_wp[:,s] = q_wp[:,s-1] + q_wp_dot*(t_wp[s]-t_wp[s-1]);
                    q_wp_dot = 0.7*(2*np.random.rand(num_states,) - 1);


            q_traj, qd_traj, qddot_traj = cubic_spline_interpolation(np.transpose(q_wp),t_end,num_snaps-1);

            X_1[i,0,:num_states] = q_traj[:,0];
            X_1[i,0,num_states:] = qd_traj[:,0];
            
            X_2[i,0,:] = X_1[i,0,:]

            X_end_1[i,0,:] = cart_pole_fkine(X_1[i,0,:], pars);
            X_end_2[i,0,:] = cart_pole_fkine(X_2[i,0,:], pars_2);

            Kp = 16;
            Kv = 8;
            F_dash = np.zeros((num_states,num_snaps))
            for j in range(num_snaps-1):
                F_dash[:,j] = qddot_traj[:,j] + Kv*(qd_traj[:,j] - X_1[i,j,num_states:]) + Kp*(q_traj[:,j] - X_1[i,j,:num_states]);
                F[i,j,:] =  (pars['m_p']+ pars['m_c'])*F_dash[1,j]

                X_1[i,j+1,:] = cart_pole_dyn(dt, X_1[i,j,:], F[i,j,:], pars);
                X_end_1[i,j+1,:] = cart_pole_fkine(X_1[i,j+1,:], pars);
                
                X_2[i,j+1,:] = cart_pole_dyn(dt, X_2[i,j,:], F[i,j,:], pars_2);
                X_end_2[i,j+1,:] = cart_pole_fkine(X_2[i,j+1,:], pars_2);



    return X_end_1, X_1, X_end_2, X_2, F
    
