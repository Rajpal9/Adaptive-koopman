## dynamics of vanderpol with correct and incorrect terms

def dynamics_vanpol(dt, num_traj, num_snaps, num_inputs, dyn_pars):
    # Here the dyanmics will be missing one of the terms
    a = dyn_pars['a']
    b = dyn_pars['b']
    c = dyn_pars['c']
    d = dyn_pars['d']
    
    # data matricies for state and data
    X_cor = np.empty((num_traj,num_snaps+1,num_states))
    X_incor = np.empty((num_traj,num_snaps+1,num_states)) 
    U = np.empty((num_traj,num_snaps,num_inputs))
    
    for i in range(num_traj):
        # Initialize matrix for each trajectory
        
        # actual dynamics
        X_cor[i,0,0] = 2*np.random.rand(1) - 1    # E [-1,1]
        X_cor[i,0,1] = 0.5*(2*np.random.rand(1) - 1) #E[-0.5,0.5]
        
        # Incorrect dynamics
        X_incor[i,0,0] = X_cor[i,0,0]
        X_incor[i,0,1] = X_cor[i,0,1]
        
        for j in range(num_snaps):
            # actual dyanmic
            # x1_dot = b*x2
            # x2_dot = -ax1 - bx2  - cu - d(x1**2)*x2 
            # for incomplete dynamics take out the x1 term
            U[i,j,:] = 0.5*(2*np.random.rand(1) - 1)
            
            # dynamics update
            x2_dot_cor = -a*X_cor[i,j,0] - b*X_cor[i,j,1] - c*U[i,j,:] -d*(X_cor[i,j,1]**2)*X_cor[i,j,1]
            x2_dot_incor = -a*X_incor[i,j,0] - 0*b*X_incor[i,j,1] - c*U[i,j,:] -d*(X_incor[i,j,1]**2)*X_incor[i,j,1]
            
            # State update
            X_cor[i,j+1,1] = X_cor[i,j,1] + x2_dot_cor*dt
            X_cor[i,j+1,0] = X_cor[i,j,0] + X_cor[i,j,1]*dt 
            
            X_incor[i,j+1,1] = X_incor[i,j,1] + x2_dot_incor*dt
            X_incor[i,j+1,0] = X_incor[i,j,0] + X_incor[i,j,1]*dt 
            
            
    return X_cor, X_incor,U   