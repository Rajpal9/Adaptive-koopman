import numpy as np

class linear_Dynamics:

    """
    this class defines the dynamics of a linear system

    """
    def __init__(self, A, B, C):
        """
        :param Ad: discrete drift matrix
        :param Bd: discrete activation matrix
        """
        self.A = A
        self.B = B
        self.C = C
        self.nz = A.shape[0]  # number of lifted states
        self.nx = C.shape[0]  # number of base states
        self.m = int(B.shape[1])  #number of control inputs

    def eval_dot(self, z, u, t = None):
        """
        computes z_k+1
        :param z: current state
        :param u: control input
        :return:
        """
        return self.A @ z + self.B @ u

    def get_linearization(self, z0, z1, u):
        """
        this computes the linearization matrix of the dynamics
        :return: A_lin, B_lin
        """

        A_lin = self.A.toarray()
        B_lin = self.B.toarray()

        z_next = self.eval_dot(z0, u)

        r_lin = z_next - z1

        return A_lin, B_lin, r_lin