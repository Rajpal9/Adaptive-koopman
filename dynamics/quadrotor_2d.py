import numpy as np
import scipy.integrate as integrate
# import dynamics.params_2d as params

#State = namedtuple('State', 'pos vel rot omega')

class Quadrotor:
    """ Quadrotor class

    state  - 1 dimensional vector but used as 13 x 1. [y, z, theta, yd, zd, theta_d]
    params - system parameters struct, arm_length, g, mass, etc.
    """

    def __init__(self, pos, orient, vel, orient_dot, params):
        """ pos = [y,z] attitude = [theta]
            """
        #print("2d_quadrotor")
        self.state = np.zeros(6)
        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = orient
        self.state[3] = vel[0]
        self.state[4] = vel[1]
        self.state[5] = orient_dot
        self.m = params['m']
        self.Ixx = params['Ixx']
        self.g = params["g"]
        self.L = params["L"]
        
        self.ode = integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')

    '''
    def world_frame(self):
        """ position returns a 3x6 matrix
            where row is [x, y, z] column is m1 m2 m3 m4 origin h
            """
        origin = self.state[0:3]
        rot = self.Rotation().T
        wHb = np.r_[np.c_[rot,origin], np.array([[0, 0, 0, 1]])]
        quadBodyFrame = params.body_frame.T
        quadWorldFrame = wHb.dot(quadBodyFrame)
        world_frame = quadWorldFrame[0:3]
        return world_frame, origin

    '''
    def get_state(self):
        return np.array([self.state[0:3],
                     self.state[3:6]]).flatten()


    def state_dot(self, t,state,par):
        # F,M = par
        t1, t2 = par
        #print("Par",type(par))
        #print("F",F)
        #print("M",M)
        y, z, theta, ydot, zdot, theta_dot = state
        #quat = np.array([qw,qx,qy,qz])

        #bRw = Quaternion(quat).as_rotation_matrix() # world to body rotation matrix
        #wRb = bRw.T # orthogonal matrix inverse = transpose
        # acceleration - Newton's second law of motion
        #accel = 1.0 / params.mass * (wRb.dot(np.array([[0, 0, F]]).T)
        #            - np.array([[0, 0, params.mass * self.g]]).T)
        # angular velocity - using quternion
        # http://www.euclideanspace.com/physics/kinematics/angularvelocity/
        #K_quat = 2.0; # this enforces the magnitude 1 constraint for the quaternion
        #quaterror = 1.0 - (qw**2 + qx**2 + qy**2 + qz**2)
        #qdot = (-1.0/2) * np.array([[0, -p, -q, -r],
        #                            [p,  0, -r,  q],
        #                            [q,  r,  0, -p],
        #                            [r, -q,  p,  0]]).dot(quat) + K_quat * quaterror * quat;

        # angular acceleration - Euler's equation of motion
        # https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
        #omega = np.array([p,q,r])
        #pqrdot = params.invI.dot( M.flatten() - np.cross(omega, params.I.dot(omega)) )
        #print("Angular acc using moment",pqrdot)
        
        ydotdot, zdotdot, theta_dotdot = np.array([0, -self.g, 0]).T  + np.dot(np.array([[-np.sin(theta)/self.m, -np.sin(theta)/self.m],
        																[np.cos(theta)/self.m,     np.cos(theta)/self.m],
        																[-self.L/self.Ixx, self.L/self.Ixx]]), np.array([t1,t2]).T)
        state_dot = np.zeros(6)
        state_dot[0]  = ydot#xdot
        state_dot[1]  = zdot#ydot
        state_dot[2]  = theta_dot#zdot
        state_dot[3]  = ydotdot#-np.array(F * np.sin(theta) / self.m) ##accel[0]
        state_dot[4]  = zdotdot#np.array(F * np.cos(theta) / self.m - self.g)#zdotdot#accel[1]
        state_dot[5]  = theta_dotdot#np.array(M / self.Ixx)##accel[2]
        #state_dot[6]  = qdot[0]
        #state_dot[7]  = qdot[1]
        #state_dot[8]  = qdot[2]
        #state_dot[9]  = qdot[3]
        #state_dot[10] = pqrdot[0]
        #state_dot[11] = pqrdot[1]
        #state_dot[12] = pqrdot[2]

        return state_dot

    def update(self, dt, t1,t2):
        #Mt = M[2]
        #prop_thrusts = params.invA.dot(np.r_[np.array([[F]]),M])
        #prop_thrusts_clamped = np.maximum(np.minimum(prop_thrusts, params.maxF/4), params.minF/4)
        ## F = np.sum(prop_thrusts_clamped)
        #M = params.A[1:].dot(prop_thrusts_clamped)
        #M = np.r_[M[:2],[Mt]]
        ## print(F)
        #F = np.clip(F, 0, 0.6)
        #M = np.clip(M, -0.05, 0.05)
        self.ode.set_initial_value(self.state,0).set_f_params([t1,t2])
        self.state = self.ode.integrate(self.ode.t + dt)


    def controller(self, state, y_ref, z_ref, y_dot_ref, z_dot_ref, y_dotdot_ref, z_dotdot_ref):
        
        y = state[0]
        z = state[1]
        phi = state[2]
        y_dot = state[3]
        z_dot = state[4]
        phi_dot = state[5]

        kp_z = 300
        kp_y = 300
        kp_phi = 2500
        
        kd_phi = 2*np.sqrt(kp_phi)
        kd_y = 2*np.sqrt(kp_y)
        kd_z = 2*np.sqrt(kp_z)

        self.F = self.m*(self.g+z_dotdot_ref+kp_z*(z_ref - z) + kd_z*(z_dot_ref - z_dot))
        phi_ref = -(1/self.g)*(y_dotdot_ref + kp_y*(y_ref - y) + kd_y*(y_dot_ref - y_dot))
        self.M = kp_phi*(phi_ref-phi)+kd_phi*(-phi_dot)

        u1 = 0.5*(self.F - self.M/self.L)
        u2 = 0.5*(self.F + self.M/self.L)

        u1_clamped = min(max(0, u1), 30)
        u2_clamped = min(max(0, u2), 30)
        return u1_clamped,u2_clamped#us,ud
