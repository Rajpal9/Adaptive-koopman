## dynamics of vanderpol with correct and incorrect terms
import numpy as np
def dynamics_lorenz(dt, num_traj, num_snaps,num_states, dyn_pars):
    # Here the dyanmics will be missing one of the terms
    sigma = dyn_pars['sigma']
    rho = dyn_pars['rho']
    beta = dyn_pars['beta']

    del_sigma = dyn_pars['del_sigma']
    del_rho = dyn_pars['del_rho']
    del_beta = dyn_pars['del_beta']

    unc_type = dyn_pars['cont_unc_type']
    delay = dyn_pars['delay']
    delay_time = dyn_pars['delay_time']
    
    # data matricies for state and data
    X_cor = np.empty((num_traj,num_snaps+1,num_states))
    X_incor = np.empty((num_traj,num_snaps+1,num_states))
    
    for i in range(num_traj):
        # Initialize matrix for each trajectory
        
        # actual dynamics
        X_cor[i,0,0] = 10*np.random.rand(1) - 1    # E [-1,1]
        X_cor[i,0,1] = 10*np.random.rand(1) - 1    # E [-1,1]
        X_cor[i,0,2] = 10*np.random.rand(1) - 1    # E [-1,1]

        
        # Incorrect dynamics
        X_incor[i,0,:] = X_cor[i,0,:]
        
        for j in range(num_snaps):

            if unc_type == 'constant':
                del_sigma = del_sigma
            elif unc_type == 'sinusoidal':
                del_sigma = del_sigma * np.sin(0.25*np.pi*(j*dt))
            elif unc_type == 'none':
                del_sigma = 0

            # actual dyanmic
            # x1_dot = sigma(x2-x1)
            # x2_dot = x1(rho-x3)-x2
            # x3_dot = x1*x2 - beta * x3
            
            # dynamics update
            x1_dot_cor = sigma*(X_cor[i,j,1] - X_cor[i,j,0])
            x2_dot_cor = X_cor[i,j,0] * (rho - X_cor[i,j,2]) - X_cor[i,j,1]
            x3_dot_cor = X_cor[i,j,0] * X_cor[i,j,1] - beta * X_cor[i,j,2]

            x1_dot_incor = (sigma + del_sigma)*(X_incor[i,j,1] - X_incor[i,j,0])
            x2_dot_incor = X_incor[i,j,0] * ((rho + del_rho) - X_incor[i,j,2]) - X_incor[i,j,1]
            x3_dot_incor = X_incor[i,j,0] * X_incor[i,j,1] - (beta + del_beta) * X_incor[i,j,2]

            
            # State update
            X_cor[i,j+1,0] = X_cor[i,j,0] + x1_dot_cor*dt
            X_cor[i,j+1,1] = X_cor[i,j,1] + x2_dot_cor*dt
            X_cor[i,j+1,2] = X_cor[i,j,2] + x3_dot_cor*dt

            X_incor[i,j+1,0] = X_incor[i,j,0] + x1_dot_incor*dt
            X_incor[i,j+1,1] = X_incor[i,j,1] + x2_dot_incor*dt
            X_incor[i,j+1,2] = X_incor[i,j,2] + x3_dot_incor*dt
            
            
    return X_cor, X_incor