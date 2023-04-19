import numpy as np

class Dynamics:

    """
    this class defines the dynamics of a linear system

    """
    def __init__(self, Ad, Bd, Hd, C):
        """
        :param Ad: discrete drift matrix
        :param Bd: discrete activation matrix
        """
        self.Ad = Ad
        self.Bd = Bd
        self.Hd = Hd
        self.C = C
        self.n = Ad.shape[0]  # number of states
        self.m = Bd.shape[1]  #number of control signals

    def eval_dot(self, z, u, t = None):
        """
        computes z_k+1
        :param z: current state
        :param u: control input
        :return:
        """
        return self.Ad @ z + self.Bd @ u + self.Hd @ np.kron(z, u)

    def get_linearization(self, z0, z1, u):
        """
        this computes the linearization matrix of the dynamics
        :return: A_lin, B_lin
        """

        A_lin = self.Ad.toarray() + self.Hd.toarray() @ np.kron(np.eye(self.n), u.reshape((self.m, 1)))
        B_lin = self.Bd.toarray() + self.Hd.toarray() @ np.kron(z0.reshape((self.n, 1)), np.eye(self.m))

        z_next = self.eval_dot(z0, u)

        r_lin = z_next - z1

        return A_lin, B_lin, r_lin

    def lift(self):
        pass