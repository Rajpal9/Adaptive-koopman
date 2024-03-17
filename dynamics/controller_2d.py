

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

        self.F = params.m*(params.g+z_dotdot_ref+kp_z*(z_ref - z) + kd_z*(z_dot_ref - z_dot))
        phi_ref = -(1/params.g)*(y_dotdot_ref + kp_y*(y_ref - y) + kd_y*(y_dot_ref - y_dot))
        self.M = kp_phi*(phi_ref-phi)+kd_phi*(-phi_dot)

        u1 = 0.5*(self.F - self.M/params.L)
        u2 = 0.5*(self.F + self.M/params.L)

        u1_clamped = min(max(0, u1), 30)
        u2_clamped = min(max(0, u2), 30)
        return u1_clamped,u2_clamped#us,ud