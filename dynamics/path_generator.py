import numpy as np

def cubic_spline_interpolation(q_,t_end, m): # time problem
    """
    Cubic Spline Interpolation

    ...

    Parameters
    ---
    q_  : Array of Position (n x Dof)
    t_end : Total time for trajectory 
    m(Optional) : Discrete Time Steps

    Returns
    ---
    q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
    """
    n = q_.shape[0]
    dof = q_.shape[1]

    q_ = np.transpose(q_)

    m = m + (m % (n-1))
    k = int(m / (n-1))
    timesteps = [np.linspace(0, 1, num = k, endpoint = False) for i in range(n-2)]
    timesteps.append(np.linspace(0, 1, num = k))

    # Generate A matrix
    A = np.zeros((dof, n, n))
    # for zero acceleration
    A[:, 0, 0] = 2
    A[:, 0, 1] = 1
    A[:, n-1, n-2] = 1
    A[:, n-1, n-1] = 2
    
    # for zero velocity
    # A[:, 0, 0] = 1
    # A[:, n-1, n-1] = 1
    for i in range(1, n-1):
        A[:, i, i - 1] = 1
        A[:, i, i] = 4
        A[:, i, i + 1] = 1

    # Generate b matrix
    y = np.zeros((dof, n))
    y[:, 0] = 3 * (q_[:, 1] - q_[:, 0])
    y[:, n-1] = 3 * (q_[:, n - 1] - q_[:, n - 2])
    y[:, 0] = 0
    # y[:, n-1] = 0
    for i in range(1, n-1):
        y[:, i] = 3 * (q_[:, i + 1] - q_[:, i - 1])

    # Solve D
    D = np.linalg.solve(A, y)

    # Calculate coefficients
    a = np.copy(q_[:, :n-1])
    b = np.copy(D[:, :n-1])
    c = np.zeros((dof, n-1))
    d = np.zeros((dof, n-1))
    for i in range(0, n-1):
        c[:, i] = 3 * (q_[:, i + 1] - q_[:, i]) - 2 * D[:, i] - D[:, i + 1]
        d[:, i] = 2 * (q_[:, i] - q_[:, i + 1]) + D[:, i] + D[:, i + 1]

    
    # Calculate Trajectories
    q = np.zeros((dof, m))
    qd = np.zeros((dof, m))
    qdd = np.zeros((dof, m))

    for j in range(n - 1):
        for i in range(len(timesteps[j])):
            t = timesteps[j][i]
            t_2 = t * t
            t_3 = t * t * t

            q[:, i + j * k] = a[:, j] + b[:, j] * t + c[:, j] * t_2 + d[:, j] * t_3
            
            
    dt = t_end/m
    for i in range(m-1):
        if i < 5:
            qd[:,i] = (q[:,i+1]-q[:,i])/dt
            qdd[:,i] = (qd[:,i+1]-qd[:,i])/dt
        else:
            qd[:,i] = (5*q[:,i+1]-3*q[:,i]-q[:,i-1]-q[:,i-2])/(8*dt)  
            qdd[:,i] = (qd[:,i+1]-3*qd[:,i]-qd[:,i-1]-qd[:,i-2])/(8*dt)
    # once this is done resample the trajectory to the given time space
    qd[:,-1] = qd[:,-2]
    qdd[:,-1] = qdd[:,-2]
    return q, qd, qdd
            
     