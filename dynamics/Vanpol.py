## dynamics of vanderpol with correct and incorrect terms
import numpy as np
def dynamics_vanpol(dt, num_traj, num_snaps,num_states, num_inputs, dyn_pars):
    # Here the dyanmics will be missing one of the terms
    a = dyn_pars['a']
    b = dyn_pars['b']
    c = dyn_pars['c']
    d = dyn_pars['d']

    del_a = dyn_pars['del_a']
    del_b = dyn_pars['del_b']
    del_c = dyn_pars['del_c']
    del_d = dyn_pars['del_d']

    unc_type = dyn_pars['cont_unc_type']
    delay = dyn_pars['delay']
    delay_time = dyn_pars['delay_time']
    
    # data matricies for state and data
    X_cor = np.empty((num_traj,num_snaps+1,num_states))
    X_incor = np.empty((num_traj,num_snaps+1,num_states)) 
    U = np.empty((num_traj,num_snaps,num_inputs))
    
    for i in range(num_traj):
        # Initialize matrix for each trajectory
        
        # actual dynamics
        X_cor[i,0,0] = 1.5*(2*np.random.rand(1) - 1)    # E [-1,1]
        X_cor[i,0,1] = 0.5*(2*np.random.rand(1) - 1) #E[-0.inaccurate_model,0.inaccurate_model]
        
        # Incorrect dynamics
        X_incor[i,0,0] = X_cor[i,0,0]
        X_incor[i,0,1] = X_cor[i,0,1]
        
        for j in range(num_snaps):
            # actual dyanmic
            # x1_dot = b*x2
            # x2_dot = -ax1 + bx2  - cu - d(x1**2)*x2
            # for incomplete dynamics take out the x1 term

            if delay == True:
                if delay_time > dt*j:
                    amp_a = 0
                    amp_b = 0
                    amp_c = 0
                    amp_d = 0
                else:
                    amp_a = del_a
                    amp_b = del_b
                    amp_c = del_c
                    amp_d = del_d
            else:
                amp_a = del_a
                amp_b = del_b
                amp_c = del_c
                amp_d = del_d

            if unc_type == 'constant':
                amp_c = amp_c
            elif unc_type == 'sinusoidal':
                amp_c = amp_c*np.sin(0.25*np.pi*(j*dt))
            elif unc_type == 'none':
                amp_c = 0


            U[i,j,:] = 0.5*(2*np.random.rand(1) - 1)
            
            # dynamics update
            x2_dot_cor = -a*X_cor[i,j,0] + b*X_cor[i,j,1] - c*U[i,j,:] -d*(X_cor[i,j,0]**2)*X_cor[i,j,1]
            x2_dot_incor = -(a+amp_a)*X_incor[i,j,0] + (b+amp_b)*X_incor[i,j,1] - (c+amp_c)*U[i,j,:] -(d+amp_d)*(X_incor[i,j,0]**2)*X_incor[i,j,1]
            
            # State update
            X_cor[i,j+1,1] = X_cor[i,j,1] + x2_dot_cor*dt
            X_cor[i,j+1,0] = X_cor[i,j,0] + X_cor[i,j,1]*dt 
            
            X_incor[i,j+1,1] = X_incor[i,j,1] + x2_dot_incor*dt
            X_incor[i,j+1,0] = X_incor[i,j,0] + X_incor[i,j,1]*dt 
            
            
    return X_cor, X_incor,U   