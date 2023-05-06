import time
import os
import numpy as np
import osqp
from scipy import sparse
import importlib


class NonlinearMPCController():
    """
    Class for nonlinear MPC with control-affine dynamics.

    Quadratic programs are solved using OSQP.
    """

    def __init__(self, dynamics, N, dt, umin, umax, xmin, xmax, Q, R, QN, solver_settings):
        """
        Initialize the nonlinear mpc class.
        :param dynamics: (AffindeDynamics) dynamics object describing system dynamics
        :param N: (int) Prediction horizon in number of timesteps
        :param dt: (float) Time interval between time steps
        :param umin: (np.array) Actuation lower bounds
        :param umax: (np.array) Actuation upper bounds
        :param xmin: (np.array) State lower bounds
        :param xmax: (np.array) State upper bounds
        :param Q: (sparse.csc_matrix) State deviation penalty matrix
        :param R: (sparse.csc_matrix) Actuation penalty matrix
        :param QN: (sparse.csc_matrix) Terminal state deviation penalty matrix
        :param xr: (np.array) Desired state, setpoint
        :param const_offset: (np.array) Constant offset of the control inputs
        :param terminal_constraint: (boolean) Constrain terminal state to be xr
        :param add_slack: (boolean) Add slack variables to state constraints
        :param q_slack: (float) Penalty value of slack terms q||s||^2, where s are the slack variables
        """

        self.dynamics_object = dynamics
        self.nz = self.dynamics_object.nz
        self.nx = self.dynamics_object.nx
        self.nu = self.dynamics_object.m
        self.dt = dt
        self.C = self.dynamics_object.C

        self.Q = Q
        self.QN = QN
        self.R = R
        self.N = N
        self.xmin = xmin
        self.xmax = xmax
        self.umin = umin
        self.umax = umax

        self.solver_settings = solver_settings

        self.prep_time = []
        self.qp_time = []
        self.comp_time = []
        self.x_iter = []
        self.u_iter = []

    def construct_controller(self, z_init, u_init, x_ref):
        """
        Construct NMPC controller.
        :param z_init: (np.array) Initial guess of z-solution
        :param u_init: (np.array) Initial guess of u-solution
        :return:
        """
        z0 = z_init[0, :]
        self.z_init = z_init
        self.u_init = u_init
        self.x_init = self.C @ z_init.T
        self.u_init_flat = self.u_init.flatten()
        self.x_init_flat = self.x_init.flatten(order='F')
        self.warm_start = np.zeros(self.nz*(self.N+1) + self.nu*self.N)

        A_lst = [np.ones((self.nz, self.nz)) for _ in range(self.N)]
        B_lst = [np.ones((self.nz, self.nu)) for _ in range(self.N)]
        r_lst = [np.ones(self.nz) for _ in range(self.N)]
        self.r_vec = np.array(r_lst).flatten()

        self.construct_objective_(x_ref)
        self.construct_constraint_vecs_(z0)
        self.construct_constraint_matrix_(A_lst, B_lst)
        self.construct_constraint_matrix_data_(A_lst, B_lst)

        # Create an OSQP object and setup workspace
        self.prob = osqp.OSQP()
        self.prob.setup(P=self._osqp_P, q=self._osqp_q, A=self._osqp_A, l=self._osqp_l, u=self._osqp_u, verbose=False,
                        warm_start=self.solver_settings['warm_start'],
                        polish=self.solver_settings['polish'],
                        polish_refine_iter=self.solver_settings['polish_refine_iter'],
                        check_termination=self.solver_settings['check_termination'],
                        eps_abs=self.solver_settings['eps_abs'],
                        eps_rel=self.solver_settings['eps_rel'],
                        eps_prim_inf=self.solver_settings['eps_prim_inf'],
                        eps_dual_inf=self.solver_settings['eps_dual_inf'],
                        linsys_solver=self.solver_settings['linsys_solver'],
                        adaptive_rho=self.solver_settings['adaptive_rho'])


    def update_solver_settings(self, solver_settings):
        """
        Update the OSQP solver settings (see OSQP documentation for detailed description of each setting)
        :param warm_start: (boolean) Warm start the solver with solution from previous timestep
        :param check_termination: (int) Frequency of checking whether the solution has converged (number of iterations)
        :param max_iter: (int) Maximum iterations allowed by the solver
        :param polish: (boolean) Execute polish step at the end of solve
        :param linsys_solver: (string) Which linear system solver to use as part of OSQP algorithm
        :return:
        """
        self.solver_settings = solver_settings
        self.prob.update_settings(warm_start=self.solver_settings['warm_start'],
                                  polish=self.solver_settings['polish'],
                                  polish_refine_iter=self.solver_settings['polish_refine_iter'],
                                  check_termination=self.solver_settings['check_termination'],
                                  eps_abs=self.solver_settings['eps_abs'],
                                  eps_rel=self.solver_settings['eps_rel'],
                                  eps_prim_inf=self.solver_settings['eps_prim_inf'],
                                  eps_dual_inf=self.solver_settings['eps_dual_inf'],
                                  linsys_solver=self.solver_settings['linsys_solver'])

    def trajectory_tracking(self, x0, z0, x_ref_traj, max_iter = 1):
        """
        compute control input for each reference position using "solve_to_convergence"
        :return:
        """
        self.xr_traj = x_ref_traj
        self.N_traj = x_ref_traj.shape[0]
        self.x_traced = np.empty((self.N_traj+1,self.nx))
        self.z_N0 = z0
        self.x_N0 = x0
        self.x_traced[0, :] = x0
        self.controls = np.empty((self.N_traj,self.nu))

        for i in range(self.N_traj):
            print("time_step",i)
            xr = self.xr_traj[i, :]
            self.solve_to_convergence(xr, self.z_N0, self.z_init, self.u_init, max_iter = max_iter, eps = 1e-3)
            self.update_initial_guess_()
            self.x_traced[i+1, :] = self.x_N0
            self.controls[i, :] = self.cur_u[0, :]


    def solve_to_convergence(self, xr, z, z_init_0, u_init_0, max_iter=1, eps=1e-3):
        """
        Run SQP-algorithm to convergence
        :param z: (np.array) Initial value of z
        :param t: (float) Initial value of t (for time-dependent dynamics)
        :param z_init_0: (np.array) Initial guess of z-solution
        :param u_init_0: (np.array) Initial guess of u-solution
        :param eps: (float) Stop criterion, normed difference of the control input sequence
        :param max_iter: (int) Maximum SQP-iterations to run
        :return:
        """
        iter = 0
        self.cur_z = z_init_0
        self.cur_u = u_init_0
        u_prev = np.zeros_like(u_init_0)
        # print("z0",z)

        while (iter == 0 or np.linalg.norm(u_prev - self.cur_u) / np.linalg.norm(u_prev) > eps) and iter < max_iter:
        # while iter < max_iter:
            t0 = time.time()
            u_prev = self.cur_u.copy()
            self.z_init = self.cur_z.copy()
            self.x_init = (self.C @ self.z_init.T)
            self.u_init = self.cur_u.copy()

            # Update equality constraint matrices:
            A_lst, B_lst = self.update_linearization_()
            # print("A_lst",A_lst)
            # print("B_lst",B_lst)

            # Solve MPC Instance
            self.update_objective_(xr)
            self.construct_constraint_vecs_(z)
            # self.construct_constraint_matrix_(A_lst, B_lst)
            # self.construct_constraint_matrix_data_(A_lst, B_lst)
            self.update_constraint_matrix_data_(A_lst, B_lst)
            t_prep = time.time() - t0

            self.solve_mpc_()
            dz = self.dz_flat.reshape(self.N + 1, self.nz)
            du = self.du_flat.reshape(self.N, self.nu)

            alpha = 1.
            self.cur_z = self.z_init + alpha * dz
            self.cur_u = self.u_init + alpha * du
            self.u_init_flat = self.u_init_flat + alpha * self.du_flat

            iter += 1
            self.comp_time.append(time.time() - t0)
            self.prep_time.append(t_prep)
            self.qp_time.append(self.comp_time[-1] - t_prep)


        self.z_N0 = self.cur_z[1, :]
        self.x_N0 = self.C @ self.z_N0.T

    # def eval(self, x, t):
    #     """
    #     Run single iteration of SQP-algorithm to get control signal in closed-loop control
    #     :param x: (np.array) Current state
    #     :param t: (float) Current time (for time-dependent dynamics)
    #     :return: u: (np.array) Current control input
    #     """
    #     t0 = time.time()
    #     z = self.dynamics_object.lift(x.reshape((1, -1)), None).squeeze()
    #     self.update_initial_guess_()
    #     self.update_objective_()
    #     A_lst, B_lst = self.update_linearization_()
    #     self.update_constraint_matrix_data_(A_lst, B_lst)
    #     self.update_constraint_vecs_(z, t)
    #     t_prep = time.time() - t0
    #
    #     self.solve_mpc_()
    #     self.cur_z = self.z_init + self.dz_flat.reshape(self.N + 1, self.nz)
    #     self.cur_u = self.u_init + self.du_flat.reshape(self.N, self.nu)
    #     self.u_init_flat = self.u_init_flat + self.du_flat
    #     self.comp_time.append(time.time() - t0)
    #     self.prep_time.append(t_prep)
    #     self.qp_time.append(self.comp_time[-1] - t_prep)
    #
    #     return self.cur_u[0, :]

    def construct_objective_(self, xr):
        """
        Construct MPC objective function
        :return:
        """
        # Quadratic objective:

        self._osqp_P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.C.T @ self.Q @ self.C),
                                          self.C.T @ self.QN @ self.C,
                                          sparse.kron(sparse.eye(self.N), self.R)], format='csc')


        # Linear objective:
        self._osqp_q = np.hstack(
            [(self.C.T @ self.Q @ (self.C @ self.z_init[:-1, :].T - xr.reshape(-1, 1))).flatten(order='F'),
             self.C.T @ self.QN @ (self.C @ self.z_init[-1, :] - xr),
             (self.R @ (self.u_init.T)).flatten(order='F')])

        # self._osqp_P.eliminate_zeros()


    def update_objective_(self, xr):
        """
        Construct MPC objective function
        :return:
        """
        self._osqp_q[:self.nz * (self.N + 1) + self.nu * self.N] = np.hstack(
            [(self.C.T @ self.Q @ (self.x_init[:, :-1] - xr.reshape(-1, 1))).flatten(order='F'),
             self.C.T @ self.QN @ (self.x_init[:, -1] - xr),
             (self.R @ (self.u_init.T)).flatten(order='F')])


    def construct_constraint_matrix_(self, A_lst, B_lst):
        """
        Construct MPC constraint matrix
        :param A_lst: (list(np.array)) List of dynamics matrices, A, for each timestep in the prediction horizon
        :param B_lst: (list(np.array)) List of dynamics matrices, B, for each timestep in the prediction horizon
        :return:
        """
        # Linear dynamics constraints:
        A_dyn = sparse.vstack((sparse.csc_matrix((self.nz, (self.N + 1) * self.nz)),
                               sparse.hstack((
                                   sparse.block_diag(A_lst), sparse.csc_matrix((self.N * self.nz, self.nz))))))
        Ax = -sparse.eye((self.N + 1) * self.nz) + A_dyn
        Bu = sparse.vstack((sparse.csc_matrix((self.nz, self.N * self.nu)),
                            sparse.block_diag(B_lst)))

        # Input constraints:
        Aineq_u = sparse.hstack(
            [sparse.csc_matrix((self.N * self.nu, (self.N + 1) * self.nz)),
             sparse.eye(self.N * self.nu)])

        # State constraints:
        Aineq_x = sparse.hstack([sparse.kron(sparse.eye(self.N + 1), self.C),
                                 sparse.csc_matrix(((self.N + 1) * self.nx, self.N * self.nu))])

        Aeq = sparse.hstack([Ax, Bu])

        self._osqp_A = sparse.vstack([Aeq, Aineq_u, Aineq_x], format='csc')
        self._osqp_A.eliminate_zeros()

    def construct_constraint_matrix_data_(self, A_lst, B_lst):
        """
        Manually build csc_matrix.data array
        :param A_lst: (list(np.array)) List of dynamics matrices, A, for each timestep in the prediction horizon
        :param B_lst: (list(np.array)) List of dynamics matrices, B, for each timestep in the prediction horizon
        :return:
        """
        C_data = [np.atleast_1d(self.C[np.nonzero(self.C[:, i]), i].squeeze()).tolist() for i in range(self.nz)]

        # State variables:
        data = []
        A_inds = []
        start_ind_A = 1
        for t in range(self.N):
            for i in range(self.nz):
                data.append(np.hstack((-np.ones(1), A_lst[t][:, i], np.array(C_data[i]))))
                A_inds.append(np.arange(start_ind_A, start_ind_A + self.nz))
                start_ind_A += self.nz + 1 + len(C_data[i])

        for i in range(self.nz):
            data.append(np.hstack((-np.ones(1), np.array(C_data[i]))))

        # Input variables:
        B_inds = []
        start_ind_B = start_ind_A + self.nz + np.nonzero(self.C)[0].size - 1
        for t in range(self.N):
            for i in range(self.nu):
                data.append(np.hstack((B_lst[t][:, i], np.ones(1))))
                B_inds.append(np.arange(start_ind_B, start_ind_B + self.nz))
                start_ind_B += self.nz + 1


        flat_data = []
        for arr in data:
            for d in arr:
                flat_data.append(d)

        self._osqp_A_data = np.array(flat_data)
        self._osqp_A_data_A_inds = np.array(A_inds).flatten().tolist()
        self._osqp_A_data_B_inds = np.array(B_inds).flatten().tolist()

    def update_constraint_matrix_data_(self, A_lst, B_lst):
        """
        Manually update csc_matrix.data array
        :param A_lst: (list(np.array)) List of dynamics matrices, A, for each timestep in the prediction horizon
        :param B_lst: (list(np.array)) List of dynamics matrices, B, for each timestep in the prediction horizon
        :return:
        """
        self._osqp_A_data[self._osqp_A_data_A_inds] = np.hstack(A_lst).flatten(order='F')
        self._osqp_A_data[self._osqp_A_data_B_inds] = np.hstack(B_lst).flatten(order='F')

    def construct_constraint_vecs_(self, z):
        """
        Construct MPC constraint vectors (lower and upper bounds)
        :param z: (np.array) Current state
        :param t: (float) Current time (for time-dependent dynamics)
        :return:
        """
        self.n_opt_x = self.nz * (self.N + 1)
        self.n_opt_x_u = self.nz * (self.N + 1) + self.nu * self.N

        dz0 = z - self.z_init[0, :]
        leq = np.hstack([-dz0, -self.r_vec])
        ueq = leq

        # Input constraints:
        u_init_flat = self.u_init.flatten()
        self.umin_tiled = np.tile(self.umin, self.N)
        self.umax_tiled = np.tile(self.umax, self.N)
        lineq_u = self.umin_tiled - u_init_flat
        uineq_u = self.umax_tiled - u_init_flat

        # State constraints:
        x_init_flat = self.x_init.flatten(order='F')
        self.xmin_tiled = np.tile(self.xmin, self.N + 1)
        self.xmax_tiled = np.tile(self.xmax, self.N + 1)
        lineq_x = self.xmin_tiled - x_init_flat
        uineq_x = self.xmax_tiled - x_init_flat


        self._osqp_l = np.hstack([leq, lineq_u, lineq_x])
        self._osqp_u = np.hstack([ueq, uineq_u, uineq_x])

    def update_constraint_vecs_(self, z):
        """
        Update MPC constraint vectors (lower and upper bounds)
        :param z: (np.array) Current state
        :param t: (float) Current time (for time-dependent dynamics)
        :return:
        """
        # Equality constraints:
        self._osqp_l[:self.nz] = -(z - self.z_init[0, :])
        self._osqp_l[self.nz:self.nz * (self.N + 1)] = -self.r_vec

        self._osqp_u[:self.nz * (self.N + 1)] = self._osqp_l[:self.nz * (self.N + 1)]

        # Input constraints:
        self._osqp_l[self.n_opt_x:self.n_opt_x_u] = self.umin_tiled - self.u_init_flat
        self._osqp_u[self.n_opt_x:self.n_opt_x_u] = self.umax_tiled - self.u_init_flat

        # State constraints:
        self._osqp_l[self.n_opt_x_u:] = self.xmin_tiled - self.x_init_flat
        self._osqp_u[self.n_opt_x_u:] = self.xmax_tiled - self.x_init_flat


    def solve_mpc_(self):
        """
        Solve the MPC sub-problem
        :return:
        """

        # print("A", self._osqp_A)
        # print("A_data",self._osqp_A_data)
        # print("A_data_indices",self._osqp_A_data_A_inds)
        # print("B_indices",self._osqp_A_data_B_inds)
        # print("P",self._osqp_P)
        # print("q",self._osqp_q)
        # print("l",self._osqp_l)
        # print("u",self._osqp_u)
        # print("z_init",self.z_init)
        # print("u_init",self.u_init)
        # print("x_init",self.x_init)
        # print("res",self.r_vec)
        self.prob.update(q=self._osqp_q, Ax=self._osqp_A_data, l=self._osqp_l, u=self._osqp_u)

        self.res = self.prob.solve()
        self.dz_flat = self.res.x[:(self.N + 1) * self.nz]
        self.du_flat = self.res.x[(self.N + 1) * self.nz:(self.N + 1) * self.nz + self.nu * self.N]
        if self.res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

    def update_initial_guess_(self):
        """
        Update the intial guess of the solution (z_init, u_init)
        :return:
        """
        z_last = self.cur_z[-1, :]
        u_new = self.cur_u[-1, :]
        z_new = self.dynamics_object.eval_dot(z_last, u_new, None)

        self.z_init[:-1, :] = self.cur_z[1:, :]
        self.z_init[-1, :] = z_new

        self.u_init[:-1, :] = self.cur_u[1:, :]
        self.u_init[-1, :] = u_new
        self.u_init_flat[:-self.nu] = self.u_init_flat[self.nu:]
        self.u_init_flat[-self.nu:] = u_new

        self.x_init = self.C @ self.z_init.T
        self.x_init_flat = self.x_init.flatten(order='F')

        # Warm start of OSQP:
        du_new = self.du_flat[-self.nu:]
        dz_last = self.dz_flat[-self.nz:]
        dz_new = self.dynamics_object.eval_dot(dz_last, du_new, None)
        self.warm_start[:self.nz*self.N] = self.dz_flat[self.nz:]
        self.warm_start[self.nz*self.N:self.nz*(self.N+1)] = dz_new
        self.warm_start[self.nz*(self.N+1):-self.nu] = self.du_flat[self.nu:]
        self.warm_start[-self.nu:] = du_new

    def update_linearization_(self):
        """
        Update the linearization of the dyanmics around the initial guess
        :return: A_lst: (list(np.array)) List of dynamics matrices, A, for each timestep in the prediction horizon
                 B_lst: (list(np.array)) List of dynamics matrices, B, for each timestep in the prediction horizon
        """
        A_lst, B_lst, r_lst = [], [], []
        for z, z_next, u in zip(self.z_init[:-1, :], self.z_init[1:, :], self.u_init):
            a, b, r = self.dynamics_object.get_linearization(z, z_next, u)
            A_lst.append(a)
            B_lst.append(b)
            r_lst.append(r)

        self.r_vec = np.array(r_lst).flatten()
        return A_lst, B_lst

    def get_state_prediction(self):
        """
        Get the state prediction from the MPC problem
        :return: Z (np.array) current state prediction
        """
        return self.cur_z

    def get_control_prediction(self):
        """
        Get the control prediction from the MPC problem
        :return: U (np.array) current control prediction
        """
        return self.cur_u