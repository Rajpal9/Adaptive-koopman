# function for definition of the paths
import numpy as np

def path_pars(t, shape):
    ## Inputs
    # t - current time,
    # t_end - time taken to complete the path,
    # c - size of the shape,
    # tilt - angle made by path with xy plane
    # rd_int = starting position for path
    # shape - shape of path

    ## Outputs
    # rd -  desired position for the path ,
    # rd_dot - desired velocity for the path,
    # rd_ddot - desired acceleration for path

    if shape == 'sinusoidal':
        #circular path
        x_ref = np.array([0.5*np.sin(2*t), 0.5*2*np.cos(2*t)])


    elif shape == 'curve1':
        x_ref = np.array([0.5*np.sin(2*t)*np.exp(-0.3*t), 0.5*(2*np.cos(2*t)*np.exp(-0.3*t) - 0.3*np.sin(2*t)*np.exp(-0.3*t))])


    elif shape == 'curve2':
        x_ref = 0.1*np.array([np.sin(t)*np.sin(6*t), np.cos(t)*np.sin(6*t)+6*np.sin(t)*np.cos(6*t)])


    elif shape == 'curve3':
        x_ref = 0.1*np.array([np.sin(t)*np.sin(8*t), np.cos(t)*np.sin(8*t)+8*np.sin(t)*np.cos(8*t)])

    elif shape == 'curve4':
        x_ref = 0.1*np.array([np.sin(t)*np.cos(2*t)*np.sin(6*t), (np.cos(t)*np.cos(2*t)*np.sin(6*t) - 2*np.sin(t)*np.sin(2*t)*np.sin(6*t) + 6*np.sin(t)*np.cos(2*t)*np.cos(6*t))])

    elif shape == "square":

        x_ref = np.array([0.2*np.sign(np.sin(2*np.pi/2*t/4)), 0*np.sign(np.cos(2*np.pi/2*t/4))])

    else:
        print('shape name not recognized')

    return x_ref

def path_cartpole(t, shape):
    ## Inputs
    # t - current time,
    # t_end - time taken to complete the path,
    # c - size of the shape,
    # tilt - angle made by path with xy plane
    # rd_int = starting position for path
    # shape - shape of path

    ## Outputs
    # rd -  desired position for the path ,
    # rd_dot - desired velocity for the path,
    # rd_ddot - desired acceleration for path

    if shape == 'curve0':
        #circular path
        x_ref = np.array([0, 0, 0, 0])


    elif shape == 'curve1':
        x_ref = np.array([0.5*np.sin(2*t)*np.exp(-0.3*t), 0.5*(2*np.cos(2*t)*np.exp(-0.3*t) - 0.3*np.sin(2*t)*np.exp(-0.3*t))])


    elif shape == 'curve2':
        x_ref = 0.1*np.array([np.sin(t)*np.sin(6*t), np.cos(t)*np.sin(6*t)+6*np.sin(t)*np.cos(6*t)])


    elif shape == 'curve3':
        x_ref = 0.1*np.array([np.sin(t)*np.sin(8*t), np.cos(t)*np.sin(8*t)+8*np.sin(t)*np.cos(8*t)])

    elif shape == 'curve4':
        x_ref = 0.1*np.array([np.sin(t)*np.cos(2*t)*np.sin(6*t), (np.cos(t)*np.cos(2*t)*np.sin(6*t) - 2*np.sin(t)*np.sin(2*t)*np.sin(6*t) + 6*np.sin(t)*np.cos(2*t)*np.cos(6*t))])

    else:
        print('shape name not recognized')

    return x_ref