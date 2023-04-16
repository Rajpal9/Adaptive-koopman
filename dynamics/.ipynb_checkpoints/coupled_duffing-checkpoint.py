import numpy as np 

def dynamics_coupled_duff(dt, num_traj, num_snaps,num_states, num_inputs, dyn_pars):
    # Here the dyanmics will be missing one of the terms
    ax = dyn_pars['ax']
    bx = dyn_pars['bx']
    cx = dyn_pars['cx']
    dx = dyn_pars['dx']

    del_ax = dyn_pars['del_ax']
    del_bx = dyn_pars['del_bx']
    del_cx = dyn_pars['del_cx']
    del_dx =dyn_pars['del_dx']

    ay = dyn_pars['ay']
    by = dyn_pars['by']
    cy = dyn_pars['cy']
    dy = dyn_pars['dy']

    del_ay = dyn_pars['del_ay']
    del_by = dyn_pars['del_by']
    del_cy = dyn_pars['del_cy']
    del_dy = dyn_pars['del_dy']

    exy = dyn_pars['exy']
    fxy = dyn_pars['fxy']
    del_exy = dyn_pars['del_exy']
    del_fxy = dyn_pars['del_fxy']
    
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
        X_cor[i,0,0] = 2*np.random.rand(1) - 1    # E [-1,1]
        X_cor[i,0,1] = 0.5*(2*np.random.rand(1) - 1) #E[-0.5,0.5]
        
        X_cor[i,0,2] = 2*np.random.rand(1) - 1    # E [-1,1]
        X_cor[i,0,3] = 0.5*(2*np.random.rand(1) - 1) #E[-0.5,0.5]
        
        # Incorrect dynamics
        X_incor[i,0,0] = X_cor[i,0,0]
        X_incor[i,0,1] = X_cor[i,0,1]
        
        X_incor[i,0,2] = 2*np.random.rand(1) - 1    # E [-1,1]
        X_incor[i,0,3] = 0.5*(2*np.random.rand(1) - 1) #E[-0.5,0.5]
        
        for j in range(num_snaps):
            # actual dyanmic
            # x1_dot = x2 = 
            # x2_dot = -ax1 - bx1**3 - cu - dx2
            # for incomplete dynamics take out the x1 term
            U[i,j,0] = 0.5*(2*np.random.rand(1) - 1)
            U[i,j,1] = 0.5*(2*np.random.rand(1) - 1)
            
            if delay == True:
                if delay_time > dt*j:
                    amp_ax = 0
                    amp_bx = 0
                    amp_cx = 0
                    amp_dx = 0
                    
                    amp_ay = 0
                    amp_by = 0
                    amp_cy = 0
                    amp_dy = 0
                    
                    amp_exy = 0
                    amp_fxy = 0
                else:
                    amp_ax = del_ax
                    amp_bx = del_bx
                    amp_cx = del_cx
                    amp_dx = del_dx
                    
                    amp_ay = del_ay
                    amp_by = del_by
                    amp_cy = del_cy
                    amp_dy = del_dy
                    
                    amp_exy = del_exy
                    amp_fxy = del_fxy
                    
            else:
                amp_ax = del_ax
                amp_bx = del_bx
                amp_cx = del_cx
                amp_dx = del_dx

                amp_ay = del_ay
                amp_by = del_by
                amp_cy = del_cy
                amp_dy = del_dy

                amp_exy = del_exy
                amp_fxy = del_fxy
                    
            
            
            if unc_type == 'constant':
                amp_cx = amp_cx
                amp_cy = amp_cy
            elif unc_type == 'sinusoidal':
                amp_cx = amp_cx*np.sin(0.25*np.pi*(j*dt))
                amp_cy = amp_cy*np.sin(0.25*np.pi*(j*dt))
            elif unc_type == 'none':
                amp_cx = 0
                amp_cy = 0
            # dynamics update
            x_ddot_cor = -(ax+amp_ax)*X_cor[i,j,0] - (bx+amp_bx)*X_cor[i,j,0]**3 - (cx+amp_cx)*U[i,j,0] -(dx+amp_dx)*X_cor[i,j,1]-(exy+amp_exy)*(X_cor[i,j,0]- X_cor[i,j,2]) - (fxy+amp_fxy)*(X_cor[i,j,0]- X_cor[i,j,2])**3
                        
            x_ddot_incor = -ax*X_incor[i,j,0] - bx*X_incor[i,j,0]**3 - cx*U[i,j,0] - dx*X_incor[i,j,1] - (exy)*(X_incor[i,j,0]-    X_incor[i,j,2]) - (fxy+amp_fxy)*(X_incor[i,j,0]- X_incor[i,j,2])**3
            
            y_ddot_cor = -(ay+amp_ay)*X_cor[i,j,2] - (by+amp_by)*X_cor[i,j,2]**3 - (cy+amp_cy)*U[i,j,1] -(dy+amp_dy)*X_cor[i,j,3]-(exy+amp_exy)*(X_cor[i,j,0]- X_cor[i,j,2]) - (fxy+amp_fxy)*(X_cor[i,j,0]- X_cor[i,j,2])**3
                        
            y_ddot_incor = -ay*X_incor[i,j,2] - by*X_incor[i,j,2]**3 - cy*U[i,j,1] - dy*X_incor[i,j,3] - (exy)*(X_incor[i,j,0]-    X_incor[i,j,2]) - (fxy+amp_fxy)*(X_incor[i,j,0]- X_incor[i,j,2])**3

            

            
            
            # State update
            X_cor[i,j+1,1] = X_cor[i,j,1] + x_ddot_cor*dt
            X_cor[i,j+1,0] = X_cor[i,j,0] + X_cor[i,j,1]*dt + (1/2)*x_ddot_cor*dt**2
            
            X_incor[i,j+1,1] = X_incor[i,j,1] + x_ddot_incor*dt
            X_incor[i,j+1,0] = X_incor[i,j,0] + X_incor[i,j,1]*dt + (1/2)*x_ddot_incor*dt**2
            
            
            
            X_cor[i,j+1,3] = X_cor[i,j,3] + y_ddot_cor*dt
            X_cor[i,j+1,2] = X_cor[i,j,2] + X_cor[i,j,3]*dt + (1/2)*y_ddot_cor*dt**2
            
            X_incor[i,j+1,3] = X_incor[i,j,3] + y_ddot_incor*dt
            X_incor[i,j+1,2] = X_incor[i,j,2] + X_incor[i,j,3]*dt + (1/2)*y_ddot_incor*dt**2
            
            
    return X_cor, X_incor,U  