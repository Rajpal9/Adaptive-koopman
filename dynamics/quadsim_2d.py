import numpy as np
import gym
from gym import spaces
#import matplotlib.pyplot as plt
#import mpl_toolkits.mplot3d.axes3d as Axes3D
#from collections import deque
from dynamics.quadrotor_2d import Quadrotor
#from math import sin, cos
# import dynamics.params_2d as params
#from Quadrotor.utils import RPYToRot,RotToQuat
#import controller_2d
import random
from control import lqr
from dynamics.TrajGen import trajGenerator, Helix_waypoints,line,random_traj_2d, lemniscate

import time

class QuadSim(gym.Env):
    def __init__(self, params):

        waypoints = random_traj_2d(np.append(np.array([0]), np.array([random.uniform(-2,2), random.uniform(-2,2)])), 2)#lemniscate()#line()#Helix_waypoints(5)#line()
        #waypoints = random_traj_2d(np.append(np.array([0]), np.array(random.sample(range(-10,10),2))), 1) #line()

        #Generate trajectory through waypoints
        traj = trajGenerator(waypoints,max_vel = 3,gamma = 100)

        #initialise simulation with given controller and trajectory
        Tmax = traj.TS[-1]
        des_state = traj.get_des_state
        
        self.pos = None
        self.attitude = 0#[0,0,0]
        self.orient_dot = 0
        
        self.t = 0
        self.ep_len = 0
        self.Tmax = Tmax
        print(Tmax)
        self.control_frequency = 100
        self.animation_frequency = 50
        self.dt = 1/self.control_frequency
        self.animation_rate = 1/self.animation_frequency
        self.control_iterations = int(self.control_frequency / self.animation_frequency)
        

        self.action_space = spaces.Box(np.array([0,0]), np.array([2, 2]))
        self.observation_space = spaces.Box(np.array([-100, -100, -100, -100, -100, -100, -100, -100]), np.array([100, 100, 100, 100, 100, 100, 100, 100]))
        
        self.des_state = des_state
        self.params = params

        #self.controller = controller
        if self.pos is None: self.pos = self.des_state(0)[1:3]
        self.Quadrotor = Quadrotor(pos=self.pos, orient=self.attitude, vel=self.des_state(0)[4:6], orient_dot=self.orient_dot, params = self.params)
        self.F = 0
        self.M = 0
        self.m = params['m']
        self.Ixx = params['Ixx']
        self.g = params["g"]
        self.L = params["L"]
        #self.eR = [0,0,0]
        #self.eW = [0,0,0]
        #print("2d_quadsim")

        
    def get_action(self):
        return self.F, self.M

    def get_thr_input(self):
        return 0.5*(self.F - self.M /self.L), 0.5*(self.F + self.M/self.L)
        
            
    
    def get_des_state(self):
        return self.des_state(self.t)
        
    #def get_att_error(self):
    #    return self.eR, self.eW
    '''
    def trajectory(self,t):
        if t < 1:
            y = 0
        else:
            y = 0.5
        z = 0
        vy = 0
        vz = 0
        ay = 0
        az = 0

        return y, z, vy, vz, ay, az                
    

    def controller(self, x, y_des, z_des, vy_des, vz_des, ay_des, az_des):
        Kp_y   = 0.4
        Kv_y   = 1.0
        Kp_z   = 0.4
        Kv_z   = 1.0
        Kp_phi = 18
        Kv_phi = 15
    [set_point[1] - state[1]],
        phi_c = -1/self.g * (ay_des + Kv_y * (vy_des - x[3]) + Kp_y * (y_des - x[0]))
        F = self.m * (self.g + az_des + Kv_z * (vz_des - x[4]) + Kp_z * (z_des - x[1]))
        M = self.Ixx * (Kv_phi * (-x[5]) + Kp_phi * (phi_c - x[2]))
        u1 = 0.5*(F - M/self.L)
        u2 = 0.5*(F + M/self.L)
    
        if u1 < 0 or u1 > 1.7658 or u2 < 0 or u2 > 1.7658:
            print(f'motor saturation {u1} {u2}')
    
        u1_clamped = min(max(0, u1), 1.7658)
        u2_clamped = min(max(0, u2), 1.7658)

        return u1_clamped, u2_clamped
        
    '''   

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

        # kp_z = 10
        # kp_y = 8
        # kp_phi = 2
        
        
        
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
    # def LQR(self, set_point, state):
    #     A_nom = np.array([[0., 0., 0., 1., 0., 0.],                         # Linearization of the true system around the origin
    #                     [0., 0., 0., 0., 1., 0.],
    #                     [0., 0., 0., 0., 0., 1.],
    #                     [0., 0., -self.g, 0., 0., 0.],
    #                     [0., 0., 0., 0., 0., 0.],
    #                     [0., 0., 0., 0., 0., 0.]])
    #     B_nom = np.array([[0., 0.],                                         # Linearization of the true system around the origin
    #                     [0., 0.],
    #                     [0., 0.],
    #                     [0., 0.],
    #                     [1./self.m, 1./self.m],
    #                     [-self.L/self.Ixx, self.L/self.Ixx]])
        
    #     Q = np.array([[750, 0, 0, 0, 0, 0], 
    #                   [0, 1000, 0, 0, 0, 0],
    #                   [0, 0, 500, 0, 0, 0],
    #                   [0, 0, 0, 750, 0, 0],
    #                   [0, 0, 0, 0, 1000, 0],
    #                   [0, 0, 0, 0, 0, 500]])
        
    #     R = np.array([[1, 0],[0, 1]])

    #     K, S, e = lqr(A_nom, B_nom, Q, R)

    #     error_matrix = np.array([[set_point[1] - state[0]],
    #                              [set_point[1] - state[1]],
    #                              [set_point[2] - state[2]],
    #                              [set_point[3] - state[3]],
    #                              [set_point[4] - state[4]],
    #                              [set_point[5] - state[5]]])
        
    #     output = np.dot(K, error_matrix)
    #     return output
        

    def step(self,action):

        total_time = self.Tmax
        total_step = total_time/self.dt
        self.t += self.dt
        self.ep_len += 1 
        
        state = self.Quadrotor.get_state()
        des_state = self.des_state(self.t)
        #des_state = self.trajectory(self.t)


        #self.U = 4*(action[0]+1)
        #self.M = np.array([[0.2*action[1]],[0.2*action[2]],[0.2*action[3]]])
        
        # self.t1 = action[0]
        # self.t2 = action[1]
        # output = self.LQR(des_state, state)
        # self.t1 = output[0][0]
        # self.t2 = output[1][0]

        t1, t2 = self.controller(state, des_state[1], des_state[2], des_state[4], des_state[5], des_state[7],  des_state[8])
        #self.t1, self.t2 = self.controller(state, des_state[0], des_state[1], des_state[2], des_state[3], des_state[4],  des_state[5])
        #print(self.Tmax)
        #if(self.t >= self.Tmax):
            #self.U, self.M = self.controller.run_hover(state, des_state,self.dt)
            #self.t = 0
            #waypoints = random_traj(state[0:3], 20) #line()

            #Generate trajectory through waypoints
            #traj = trajGenerator(waypoints,max_vel = 10,gamma = 1e6)

            #initialise simulation with given controller and trajectory
            #self.Tmax = traj.TS[-1]
            #des_state = traj.get_des_state
            #self.U, self.M = self.controller.run(state, des_state)
            #print("Action",self.U,self.M)
        
        #else:
            #self.U, self.M = self.controller.run(state, des_state)
    
        
        #self.U = np.clip(self.U, 0, 0.6)
        #self.M = np.clip(self.M, -0.05, 0.05)
        #print(self.U, self.M)
        self.Quadrotor.update(self.dt, t1, t2)
        
        state = self.Quadrotor.get_state()
        
        #current Rotation from body frame to world
        #bRw = RPYToRot(state[6],state[7],state[8]).T
        #normal vector to body frame
        #ZB = bRw[:,2]


        #current acceleration
        # curr_acc_y = -((self.t1+self.t2)*np.sin(state[2]))/self.m 
        # curr_acc_z = ((self.t1+self.t2)*np.cos(state[2]))/self.m - self.g
        # curr_acc = [curr_acc_y,curr_acc_z]
        #acceleration error
    
        pos_err = np.array(state[0:2] - des_state[1:3])#0-3
        vel_err = np.array(state[3:5] - des_state[4:6])#3-6
        # acc_err = np.array(curr_acc - des_state[7:9])#6-9
        
        #quat = RotToQuat(bRw)
        
        Obs = np.concatenate([pos_err, vel_err,  np.array([state[2]]), np.array([state[5]])]).flatten()#acc_err,
    
        # acc_l2 = np.linalg.norm(acc_err)
        pos_l2 = np.linalg.norm(pos_err) 
        vel_l2 = np.linalg.norm(vel_err)
        yaw_error = abs(state[2])
    
        r_pos = np.exp(-(1/0.01*pos_l2)) * 0.6 #pos_l2 * (-5.0)#
        r_vel = np.exp(-(1/0.05*vel_l2)) * 0.1 #vel_l2 * (-2)#
        # r_acc = np.exp(-(1/1*acc_l2)) * 0.1 #acc_l2 * (-2)#
        r_yaw = np.exp(-(1/(5/180*np.pi)*yaw_error)) * 0.2 #yaw_error * (-0.3)#
        
        #extra_bonus = self.bonus_reward_to_achieve_goal(pos_err)
        
        #reward_velocity_towards_goal = 0.0
        #if pos_l2 > 0.007:
        #    reward_velocity_towards_goal += self.reward_velocity_towards_goal(error_xyz=pos_err, velocity=state[3:6])
    
        rewards = r_pos + r_vel +  r_yaw #r_acc ++ 5.0 + reward_velocity_towards_goal + extra_bonus


        #Termination condition and reward
        done = False
        reward_terminate = 0

        # if (pos_l2 > 0.5*3 or yaw_error > 5*3/180*np.pi): 
            #print("Exceeded Error threshold")    
            # done = True
            # reward_terminate = -1
        
        if self.t >= total_time+1 and not done:
            done =True        
        
        #if self.ep_len == 1e+6:
            #done =True

        if not done:
            Reward = rewards
            
        else:
            Reward = reward_terminate
        
        #print(U,M) 

        return Obs, Reward, done, {}   



    def reset(self):
        self.t = 0
        self.ep_len = 0
        waypoints = random_traj_2d(np.append(np.array([0]), np.array([random.uniform(-2,2), random.uniform(-2,2)])), 3)#lemniscate()#line()#Helix_waypoints(5)#
        #waypoints = random_traj_2d(np.append(np.array([0]), np.array(random.sample(range(-10,10),2))), 1) #line()

        #Generate trajectory through waypoints
        traj = trajGenerator(waypoints,max_vel = 3,gamma = 100)

        #initialise simulation with given controller and trajectory
        self.Tmax = traj.TS[-1]
        #print(self.Tmax)
        self.des_state = traj.get_des_state
        self.pos = None
        self.attitude = 0#[0,0,0]
        self.orient_dot = 0
        if self.pos is None: self.pos = self.des_state(0)[1:3]
        self.Quadrotor = Quadrotor(pos=self.pos, orient=self.attitude, vel=self.des_state(0)[4:6], orient_dot=self.orient_dot, params = self.params)
        
        state = self.Quadrotor.get_state()
        des_state = self.des_state(self.t)
        self.F = 0
        self.M = 0
        #self.U = 0
        #bRw = RPYToRot(state[6],state[7],state[8]).T
        #ZB = bRw[:,2]
        #curr_acc = self.U*ZB/params.mass - self.g*np.array([0,0,1])
        #acceleration error
        #current acceleration
        # curr_acc_y = -((self.t1+self.t2)*np.sin(state[2]))/self.m 
        # curr_acc_z = ((self.t1+self.t2)*np.cos(state[2]))/self.m - self.g
        # curr_acc = [curr_acc_y,curr_acc_z]
        #acceleration error
    
        pos_err = np.array(state[0:2] - des_state[1:3])#0-3
        vel_err = np.array(state[3:5] - des_state[4:6])#3-6
        # acc_err = np.array(curr_acc - des_state[7:9])#6-9
        
        #quat = RotToQuat(bRw)        
        Obs = np.concatenate([pos_err, vel_err, np.array([state[2]]), np.array([state[5]])]).flatten()#acc_err, 
        return Obs
                    

    
