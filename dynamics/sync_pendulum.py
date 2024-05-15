import numpy as np



def FK_solver(x, u, pars,sensor_noise=False, SNR_DB = 0, i=0):
    """
    dt = timestep
    x = [x, theta, x_dt, theta_dt];
    F = Force Input
    pars = system parameters
    
    """
    dt = pars['dt']
    m = pars["m"]    # mass of pendulum
    l = pars["l"]        # length of pendulum            
    g = 9.81             # acceleration of gravity
    J = pars["J"]  # MOI of Pendulum
    b = pars["b"]    # damping coefficient of the relative speed
    k = pars["k"]    # spring stiffness
    gamma = pars['gamma']  #damping coefficient of the absolute speed
    N  = pars['N'] #number of pendulum
    L = pars['L'] # graph laplacian defining the system
    f_origin = pars['f_origin']
    d = pars['d'] #array defining the input dynamics


    ## dynamics of the system
    G = np.array([[0],[1]])
    K = np.array([k,b]).reshape((1,-1))

    pos = x[::2].reshape((N,-1))
    vel = x[1::2].reshape((N,-1))

    if sensor_noise:
        noise = (np.random.normal(0,1,2*N)).reshape((2*N,1))
    else:
        noise = 0

    if pars['uncertainty']=='constant':
        uncertainty_factor = pars['amp']
    elif pars['uncertainty'] == 'periodic':
        uncertainty_factor = pars['amp']*np.sin(2*np.pi*pars['freq']*i*dt)
    else:
        uncertainty_factor = 0

    #drift (uncoupled) dynamics
    if not f_origin:
        f_drift = np.kron(vel,np.array([[1],[0]])) + np.kron(-m*g*l/J*np.sin(pos) - (gamma/J)*vel, np.array([[0],[1]]))
    else:
       f_drift =  np.kron(vel,np.array([[1],[0]])) + np.kron(m*g*l/J*np.sin(pos) - (gamma/J)*vel, np.array([[0],[1]]))
    A_inter = -np.kron(L,G @ K)/J
    controlled_input = np.kron(d,G)/J*(u+uncertainty_factor)


    dxdt = (f_drift + (A_inter @ x.reshape((2*N,-1))) + controlled_input)* (1+noise*10**(-SNR_DB/20)) + uncertainty_factor

    x_next = x + dt*dxdt.reshape((2*N,))
    
    return x_next

from scipy.integrate import solve_ivp
    
    
def sync_pendulum_data_gen_multi(num_traj,num_snaps, pars, pars_new,sensor_noise, SNR_DB):
    
    """
    A function for generating the data for the two cart pole systems
    dt = time step
    num_traj = number of trajectories
    num_snap = number of snaps in each trajectory
    robot = 1st cart pole
    robot_2 = 2nd cart pole
    controller = random or model partitioning controller (controller)
    """
    n = pars['num_states']
    m = pars['num_inputs']
    dt = pars['dt']
    F_max = pars['F_max']

    X_1 = np.zeros((num_traj, num_snaps, n))
    X_2 = np.zeros((num_traj, num_snaps, n))

    U = np.zeros((num_traj, num_snaps-1, m))


    for i in range(num_traj):
        X_1[i,0,0::2] = 20*(2*np.random.rand(1,) - 1)
        X_1[i,0,1::2] = (2*np.random.rand(1,) - 1)
        X_2[i,0,:] = X_1[i,0,:]

        num_list = np.linspace(1,20)
        x1_seed = np.random.choice(num_list)
        x2_seed = np.random.choice(num_list)

        for j in range(num_snaps-1):
            if pars["F_type"]== 'random':
                U[i,j,:] = F_max*(2*np.random.rand(m,1) - 1).reshape(m,) #input torques

            elif pars['F_type'] == 'sinusoidal':
                U[i,j,:] = F_max*(np.sin(x1_seed*np.pi*j*dt)*np.cos(x2_seed*np.pi*j*dt)).reshape(m,)

            X_1[i,j+1,:] =  FK_solver(X_1[i,j,:], U[i,j,:],pars)
            X_2[i,j+1,:] =  FK_solver(X_2[i,j,:], U[i,j,:],pars_new,sensor_noise, SNR_DB, j)

    return X_1, X_2, U
    
