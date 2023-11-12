import roboticstoolbox as rtb
import numpy as np


def robot_selector(robot_name, params):
    """
    Function for generating the required robot
    
    Inputs
    Robot_name  = Name of the Robot
    params = list of params - mass, inertia, link length


    """    
    if robot_name == "3R_2D":
        robot = robot_3R_2D(params)
        
    elif robot_name == "3R_3D":
        robot = robot_3R_3D(params)
        
    elif robot_name == "4R_2D":
        robot = robot_4R_2D(params)
    
    elif robot_name == "4R_3D":
        robot = robot_4R_3D(params)
    
    elif robot_name == "5R_2D":
        robot = robot_5R_2D(params)
    
    elif robot_name == "5R_3D":
        robot = robot_5R_3D(params)

    elif robot_name == "2R_2D":
        robot = robot_2R_2D(params)
    else:
        print("Invalid Robot Name")
        return
        
        
    return robot
    
    
def robot_2R_2D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0,  d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),               
            ], name="2R_2D") 
    
    
    return robot

def robot_3R_2D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0,  d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),               
            ], name="3R_2D") 
    
    
    return robot

def robot_3R_3D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0,  d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = np.pi/2, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),               
            ], name="3R_3D") 
    
    
    return robot

def robot_4R_2D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I)
            ], name="4R_2D") 
    
    
    return robot

def robot_4R_3D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = np.pi/2, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I)
            ], name="4R_3D") 
    
    
    return robot


def robot_5R_2D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I)
            ], name="5R_2D") 
    
    
    return robot

def robot_5R_3D(params):
    m = params['m']
    I = params['I']
    l = params['l']
    
    
    robot = rtb.robot.DHRobot([rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0, r = [l/2,0,0], m = m, I = I  ),
                               rtb.robot.DHLink(a = l, alpha = np.pi/2, d = 0, theta = 0, r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = np.pi/2, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I),
                               rtb.robot.DHLink(a = l, alpha = 0, d = 0,theta = 0,r = [l/2,0,0], m = m, I = I)
            ], name="5R_3D") 
    
    
    return robot
