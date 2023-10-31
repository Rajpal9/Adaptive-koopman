import numpy as np

def Vander_adapt(dt,x, u, num_states, dyn_pars):
    a = dyn_pars['a']
    b = dyn_pars['b']
    c = dyn_pars['c']
    d = dyn_pars['d']

    del_a = dyn_pars['del_a']
    del_b = dyn_pars['del_b']
    del_c = dyn_pars['del_c']
    del_d = dyn_pars['del_d']

    x_next_new = np.empty((num_states))
    x_next_old = np.empty((num_states))

    # dynamics update
    x2_dot_new = -(a+del_a)*x[0] + (b+del_b)*x[1] - (c+del_c)*u -(d+del_d)*x[0]**2*x[1]
    x_next_new[1] = x[1] + x2_dot_new*dt
    x_next_new[0] = x[0] + x[1]*dt + (1/2)*x2_dot_new*dt**2

    x2_dot_old = -a*x[0] + b*x[1] - c*u -d*x[0]**2*x[1]
    x_next_old[1] = x[1] + x2_dot_old*dt
    x_next_old[0] = x[0] + x[1]*dt + (1/2)*x2_dot_old*dt**2

    return x_next_old, x_next_new