import numpy as np

def dynamics_3R(dt,num_traj,num_snaps,num_states,num_inputs, robot_pars):
  #define parameters
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
  X = np.empty((num_traj,num_snaps+1,num_states)) # cartesian state matrix
  U = np.empty((num_traj,num_snaps,num_inputs)) # input matrix
  x_end = np.empty((num_traj,num_snaps+1,2)) # position of end effector

  for i in range(num_traj):
    # initialize the values for the trajectory
      # joint angles
    X[i,0,0:3] = np.pi*(2*np.random.rand(1,1,3)-1) # theta values
      # joint velocities
    X[i,0,3:6] = 0.1*(2*np.random.rand(1,1,3)-1) # theta values
    x_end[i,0,0]= a1*np.cos(X[i,0,0])+a2*np.cos(X[i,0,0]+X[i,0,1])+a3*np.cos(X[i,0,0]+X[i,0,1]+X[i,0,2])
    x_end[i,0,1]= a1*np.sin(X[i,0,0])+a2*np.sin(X[i,0,0]+X[i,0,1])+a3*np.sin(X[i,0,0]+X[i,0,1]+X[i,0,2])


    for j in range(num_snaps):

        #define thetas and theta_dots for convenience
        th1 = X[i,j,0]
        th2 = X[i,j,1]
        th3 = X[i,j,2]
        th1_dot = X[i,j,3]
        th2_dot = X[i,j,4]
        th3_dot = X[i,j,5]

        theta_dot = X[i,j,3:6].reshape(num_inputs,1)


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

        # inputs
        U[i,j,:] = 1*(2*np.random.rand(1,1,3)-1)
        tau = np.array([[U[i,j,0]],[U[i,j,1]],[U[i,j,2]]])

        # theta evolution
        th_ddot = np.matmul(np.linalg.inv(M),(tau-G-np.matmul(C,theta_dot)-(fd*theta_dot))).reshape(1,1,3)
        X[i,j+1,3:6] = X[i,j,3:6] + th_ddot*dt
        X[i,j+1,0:3] = X[i,j,0:3] + X[i,j,3:6]*dt +(1/2)*th_ddot*dt**2

        x_end[i,j+1,0]= a1*np.cos(X[i,j+1,0])+a2*np.cos(X[i,j+1,0]+X[i,j+1,1])+a3*np.cos(X[i,j+1,0]+X[i,j+1,1]+X[i,j+1,2])
        x_end[i,j+1,1]= a1*np.sin(X[i,j+1,0])+a2*np.sin(X[i,j+1,0]+X[i,j+1,1])+a3*np.sin(X[i,j+1,0]+X[i,j+1,1]+X[i,j+1,2])

  return x_end,X,U