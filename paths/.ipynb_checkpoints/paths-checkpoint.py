# function for definition of the paths
import numpy as np

def path_pars(t,t_end,c,tilt,rd_init, shape):
    ## Inputs
    # t - current time, 
    # t_end - time taken to complete the path,
    # c - size of the shape, 
    # tilt - angle made by path with xy plane
    # rd_int = starting position for path 
    # shape - shape of path

    ## Outputs
    # rd -  desired position for the path ,
    # rd_dot - desired velocity for the path, 
    # rd_ddot - desired acceleration for path
    
    if shape == 'cardioid':
    #cardioid path
        n = t_end/10
        rd = np.array([2*c*np.sin(0.2*np.pi*t/n)*np.cos(tilt)-c*np.sin(0.4*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                       2*c*np.cos(0.2*np.pi*t/n)             -c*np.cos(0.4*np.pi*t/n)+rd_init[1]-c,
                       2*c*np.sin(0.2*np.pi*t/n)*np.sin(tilt)-c*np.sin(0.4*np.pi*t/n)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([2*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt)- c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.cos(tilt),
                          -2*c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n)             + c*(0.4*np.pi/n)*np.sin(0.4*np.pi*t/n),
                           2*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt)- c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.sin(tilt)])
        
        rd_ddot =np.array([-2*c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.cos(tilt) + c*((0.4*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.cos(tilt),
                           -2*c*((0.2*np.pi/n)**2)*np.cos(0.2*np.pi*t/n)              + c*((0.4*np.pi/n)**2)*np.cos(0.4*np.pi*t/n),
                           -2*c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.sin(tilt) + c*((0.4*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.sin(tilt)])
    
    elif shape == 'adobe':
        # epicycloid path
        n = t_end/10

        rd = np.array([-3*c*np.sin(0.2*np.pi*t/n)*np.cos(tilt) + 2*c*np.sin(0.4*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                        3*c*np.cos(0.2*np.pi*t/n)              + 2*c*np.cos(0.4*np.pi*t/n)+rd_init[1]-5*c,
                       -3*c*np.sin(0.2*np.pi*t/n)*np.sin(tilt) + 2*c*np.sin(0.4*np.pi*t/n)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([-3*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt) + 2*c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.cos(tilt),
                           -3*c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n)              - 2*c*(0.4*np.pi/n)*np.sin(0.4*np.pi*t/n),
                           -3*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt) + 2*c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.sin(tilt)])
    
        rd_ddot =np.array([-3*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.cos(tilt)-2*c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.cos(tilt),
                    +3*c*((0.2*np.pi)**2)*np.cos(0.2*np.pi*t)-2*c*((0.4*np.pi)**2)*np.cos(0.4*np.pi*t),
                    -3*c*((0.2*np.pi)**2)*np.sin(0.2*np.pi*t)*np.sin(tilt)-2*c*((0.4*np.pi)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt)])
    


    elif shape == 'hyp':
        # hypocycliod path
        n = t_end/10

        rd = np.array([2*c*np.sin(0.2*np.pi*t/n)*np.cos(tilt) - c*np.sin(0.8*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                      2*c*np.cos(0.2*np.pi*t/n)               - c*np.cos(0.8*np.pi*t/n)+rd_init[1]-c,
                      2*c*np.sin(0.2*np.pi*t/n)*np.sin(tilt)  - c*np.sin(0.8*np.pi*t/n)*np.sin(tilt)+rd_init[2]])

        
        rd_dot = np.array([2*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt) - c*(0.8*np.pi/n)*np.cos(0.8*np.pi*t/n)*np.cos(tilt),
                          -2*c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n)              + c*(0.8*np.pi/n)*np.sin(0.8*np.pi*t/n),
                           2*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt) - c*(0.8*np.pi/n)*np.cos(0.8*np.pi*t/n)*np.sin(tilt)])
        
        
        rd_ddot =np.array([-2*c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.cos(tilt) + c*((0.8*np.pi/n)**2)*np.sin(0.8*np.pi*t/n)*np.cos(tilt),
                           -2*c*((0.2*np.pi/n)**2)*np.cos(0.2*np.pi*t/n)              + c*((0.8*np.pi/n)**2)*np.cos(0.8*np.pi*t/n),
                           -2*c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.sin(tilt) + c*((0.8*np.pi/n)**2)*np.sin(0.8*np.pi*t/n)*np.sin(tilt)])
        


    elif shape == 'petal':
        n = t_end/10

        rd = np.array([c*np.sin(0.2*np.pi*t/n)*np.cos(tilt) - c*np.sin(0.6*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                       c*np.cos(0.2*np.pi*t/n)              + c*np.cos(0.6*np.pi*t/n)+rd_init[1]-2*c,
                       c*np.sin(0.2*np.pi*t/n)*np.sin(tilt) - c*np.sin(0.6*np.pi*t/n)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt) - c*(0.6*np.pi/n)*np.cos(0.6*np.pi*t/n)*np.cos(tilt),
                          -c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n)              - c*(0.6*np.pi/n)*np.sin(0.6*np.pi*t/n),
                           c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt) - c*(0.6*np.pi/n)*np.cos(0.6*np.pi*t/n)*np.sin(tilt)])
    
        rd_ddot =np.array([-c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.cos(tilt) + c*((0.6*np.pi/n)**2)*np.sin(0.6*np.pi*t/n)*np.cos(tilt),
                           -c*((0.2*np.pi/n)**2)*np.cos(0.2*np.pi*t/n)              - c*((0.6*np.pi/n)**2)*np.cos(0.6*np.pi*t/n),
                           -c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.sin(tilt) + c*((0.6*np.pi/n)**2)*np.sin(0.6*np.pi*t/n)*np.sin(tilt)])

    elif shape == 'tricuspid':

        n = t_end/10
        
        rd = np.array([-2*c*np.sin(0.2*np.pi*t/n)*np.cos(tilt) + c*np.sin(0.4*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                        2*c*np.cos(0.2*np.pi*t/n)              + c*np.cos(0.4*np.pi*t/n)+rd_init[1]-3*c,
                       -2*c*np.sin(0.2*np.pi*t/n)*np.sin(tilt) + c*np.sin(0.4*np.pi*t/n)*np.sin(tilt)+rd_init[2]])

        rd_dot = np.array([-2*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt) + c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.cos(tilt),
                           -2*c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n)              - c*(0.4*np.pi/n)*np.sin(0.4*np.pi*t/n),
                           -2*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt) + c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.sin(tilt)])
        
        rd_ddot =np.array([-2*c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.cos(tilt) - c*((0.4*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.cos(tilt),
                           +2*c*((0.2*np.pi/n)**2)*np.cos(0.2*np.pi*t/n)              - c*((0.4*np.pi/n)**2)*np.cos(0.4*np.pi*t/n),
                           -2*c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.sin(tilt) - c*((0.4*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.sin(tilt)])

    elif shape == 'circle':
        # circle
        n = t_end/10
        rd = np.array([c*np.sin(0.2*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                       c*np.cos(0.2*np.pi*t/n)+rd_init[1]-c,
                       c*np.sin(0.2*np.pi*t/n)*np.sin(tilt)+rd_init[2]])

 
        rd_dot = np.array([ c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt),
                           -c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n),
                            c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt)])
        
        
        rd_ddot =np.array([-c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.cos(tilt),
                           -c*((0.2*np.pi/n)**2)*np.cos(0.2*np.pi*t/n),
                           -c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.sin(tilt)])

    
    elif shape == 'helix':
        # circle
        tilt = 0
        max_hieght = 0.2
        n_rounds = 6
        n = (t_end)/(10*n_rounds)
        rd = np.array([c*np.sin(0.2*np.pi*t/n)+rd_init[0],
                       c*np.cos(0.2*np.pi*t/n)+rd_init[1]-c,
                       rd_init[2]+ (max_hieght-rd_init[2])*t/t_end])

 
        rd_dot = np.array([ c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt),
                           -c*(0.2*np.pi/n)*np.sin(0.2*np.pi*t/n),
                            (max_hieght-rd_init[2])/t_end])
        
        
        rd_ddot =np.array([-c*((0.2*np.pi/n)**2)*np.sin(0.2*np.pi*t/n)*np.cos(tilt),
                           -c*((0.2*np.pi/n)**2)*np.cos(0.2*np.pi*t/n),
                           0])

    #star shape
    elif shape == 'star':
        n = t_end/10

        rd = np.array([c*np.sin(0.6*np.pi*t/n)*np.cos(tilt)-2*c*np.sin(0.4*np.pi*t/n)*np.cos(tilt)+rd_init[0],
                       c*np.cos(0.6*np.pi*t/n)             +2*c*np.cos(0.4*np.pi*t/n)+rd_init[1]-3*c,
                       c*np.sin(0.6*np.pi*t/n)*np.sin(tilt)-2*c*np.sin(0.4*np.pi*t/n)*np.sin(tilt)+rd_init[2]])
        
        rd_dot = np.array([c*(0.6*np.pi/n)*np.cos(0.6*np.pi*t/n)*np.cos(tilt) -2*c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.cos(tilt),
                          -c*(0.6*np.pi/n)*np.sin(0.6*np.pi*t/n)              -2*c*(0.4*np.pi/n)*np.sin(0.4*np.pi*t/n),
                            c*(0.6*np.pi/n)*np.cos(0.6*np.pi*t/n)*np.sin(tilt)-2*c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n)*np.sin(tilt)])
    
        rd_ddot =np.array([-c*((0.6*np.pi/n)**2)*np.sin(0.6*np.pi*t/n)*np.cos(tilt)+2*c*((0.4*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.cos(tilt),
                           -c*((0.6*np.pi/n)**2)*np.cos(0.6*np.pi*t/n)             -2*c*((0.4*np.pi/n)**2)*np.cos(0.4*np.pi*t/n),
                           -c*((0.6*np.pi/n)**2)*np.sin(0.6*np.pi*t/n)*np.sin(tilt)+2*c*((0.4*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.sin(tilt)])
        

    elif shape == 'lissajous':
        n = t_end/10

        rd = np.array([4*c*np.sin(0.2*np.pi*t/n)*np.cos(tilt) + rd_init[0],
                       3*c*np.sin(0.4*np.pi*t/n)+rd_init[1],
                       4*c*np.sin(0.2*np.pi*t/n)*np.sin(tilt) + rd_init[2]])

        rd_dot = np.array([4*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.cos(tilt),
                           3*c*(0.4*np.pi/n)*np.cos(0.4*np.pi*t/n),
                           4*c*(0.2*np.pi/n)*np.cos(0.2*np.pi*t/n)*np.sin(tilt)])
    
        rd_ddot = np.array([-4*c*((0.2*np.pi/n)**2)*np.sin(0.4*np.pi*t/n)*np.cos(tilt),
                           -3*c*((0.4*np.pi/n)**2)*np.sin(0.2*np.pi*t/n),
                           -4*c*((0.2*np.pi/n)**2)*np.sin(0.4*np.pi*t)*np.sin(tilt)])
        

    elif shape == 'line':
        

        rd = np.array([10*c*t/t_end +  rd_init[0],
                      rd_init[1],
                      rd_init[2]])

        rd_dot = np.array([10*c/t_end,
                           0,               
                           0])
    
        rd_ddot = np.array([0,
                            0,
                            0])


    else:
        print('shape name not recognized')

    return rd, rd_dot, rd_ddot