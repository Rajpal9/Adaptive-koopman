import numpy as np
import random 
#from collections import namedtuple

#DesiredState = namedtuple('DesiredState', 'pos vel acc jerk yaw yawdot')

def polyder(t, k = 0, order = 10):
    if k == 'all':
        terms = np.array([polyder(t,k,order) for k in range(1,5)])
    else:
        terms = np.zeros(order)
        coeffs = np.polyder([1]*order,k)[::-1]
        pows = t**np.arange(0,order-k,1)
        terms[k:] = coeffs*pows
    return terms

def Hessian(T,order = 10,opt = 4):
    n = len(T)
    Q = np.zeros((order*n,order*n))
    for k in range(n):
        m = np.arange(0,opt,1)
        for i in range(order):
            for j in range(order):
                if i >= opt and j >= opt:
                    pow = i+j-2*opt+1
                    Q[order*k+i,order*k+j] = 2*np.prod((i-m)*(j-m))*T[k]**pow/pow
    return Q

def Circle_waypoints(n,Tmax = 2*np.pi):
    t = np.linspace(0,Tmax, n)
    x = 1+0*t
    y = 0+2*np.cos(t)
    z = 0+2*np.sin(t)
    
    return np.stack((x, y, z), axis=-1)

def Helix_waypoints(n,Tmax = 2*np.pi):

    t = np.linspace(0, Tmax, n)
    x = 0+2*np.cos(t)
    y = 0+2*np.sin(t)
    z = t/Tmax*2

    return np.stack((x, y, z), axis=-1)

def line():
    #t = np.linspace(0, Tmax, 2)
    x = np.linspace(0, 10, 10)
    y = np.linspace(0, 10, 10)
    z = np.linspace(0, 10, 10)

    return np.stack((x, y, z), axis=-1)

def random_traj_3d(init_pos, N):
    pts = [init_pos]
    for i in range(N):
        pts = np.vstack((pts, np.array([random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0, 2)])))
        #pts = np.vstack((pts, np.array(random.sample(range(-10,10),2))))
        #pts = np.append(pts, np.append(np.random.randint(-10,10,2),np.array([3])).reshape()
    return pts 
    
def random_traj_2d(init_pos, N):
    pts = [init_pos]
    for i in range(N):
        pts = np.vstack((pts, np.append(np.array([0]), np.array(random.sample(range(-10,10),2)))))
        #pts = np.vstack((pts, np.array(random.sample(range(-10,10),2))))
        #pts = np.append(pts, np.append(np.random.randint(-10,10,2),np.array([3])).reshape()
    return pts 
    
def lemniscate():
    pts = np.array([[0,0,0],[0,1,1],[0,2,0],[0,1,-1],[0,0,0],[0,-1,1],[0,-2,0],[0,-1,-1],[0,0,0]])
    return pts      

def lemniscate_2d():
    pts = np.array([[0,0],[1,1],[2,0],[1,-1],[0,0],[-1,1],[-2,0],[-1,-1],[0,0]])
    return pts 

def adobe(n,Tmax = 2*np.pi):
    c = 0.3
    t = np.linspace(0,Tmax, n)
    x = 1+0*t
    y = 0 + 2*c*np.sin(2*t) - 3*c*np.sin(t)
    z = 0 + 2*c*np.cos(2*t) + 3*c*np.cos(t) - 5*c
    
    return np.stack((x, y, z), axis=-1)

def star(n,Tmax = 2*np.pi):
    c = 0.5
    t = np.linspace(0,Tmax, n)
    x = 1+0*t
    y = 0 + 2*c*np.cos(3*t) - 2*c*np.sin(2*t)
    z = 0 + 2*c*np.cos(3*t) + 2*c*np.cos(2*t) - 3*c
    
    return np.stack((x, y, z), axis=-1)


