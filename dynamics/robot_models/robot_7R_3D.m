function robot = robot_7R_3D
    % parameter
    %link length
    l1 = 0.33; % l1 = 0.33
    l2 = 0.33;
    l3 = 0.33;
    l4 = 0.33; % l1 = 0.33
    l5 = 0.33;
    l6 = 0.33;
    l7 = 0.33;
    l8 = 0.33;
    l9 = 0.33;
    
    m1 = 1; %0.1
    m2 = 1;
    m3 = 1;
    m4 = 1;
    m5 = 1;
    m6 = 1;
    m7 = 1;
    m8 = 1;
    m9 = 0;
    
    % link inertias
    I1 = [ 1 1 1 0 0 0]; %[Ixx Iyy Izz Iyz Ixz Ixy]. %[ 1 1 1 0 0 0]
    I2 = [ 1 1 1 0 0 0];
    I3 = [ 1 1 1 0 0 0];
    I4 = [ 1 1 1 0 0 0];
    I5 = [ 1 1 1 0 0 0];
    I6 = [ 1 1 1 0 0 0];
    I7 = [ 1 1 1 0 0 0];
    I8 = [ 1 1 1 0 0 0];
    I9 = [0,0,0,0,0,0];
    % COM
    COM1 = [l1/2 0 0]; % [x y z] in body frame
    COM2 = [l2/2 0 0]; % [x y z] in body frame
    COM3 = [l3/2 0 0]; % [x y z] in body frame
    COM4 = [l4/2 0 0]; % [x y z] in body frame
    COM5 = [l5/2 0 0]; % [x y z] in body frame
    COM6 = [l6/2 0 0]; % [x y z] in body frame
    COM7 = [l7/2 0 0]; % [x y z] in body frame
    COM8 = [l8/2 0 0];
    COM9 = [l9/2 0 0];
    % dh params
    % [a alpha d theta]
    % dhparams = [ 0 0 0 0;
    %             l1 0 0 0;
    %             l2 pi/2 0 pi;
    %             l3 0 0 0;
    %             l4 pi/2 0 pi;
    %             l5 0  0  0];
    
    
    dhparams = [ 0 0 0 0;
                 0 0 0.36 0;
                 0 pi/2 0 0;
                 0 -pi/2 0.42 0;
                 0 -pi/2 0 0;
                 0  pi/2 0.40 0;
                 0  pi/2  0  0;
                 0 -pi/2 0 0
                 0  0    0.126 0];
    
    
    %% with mass var
    % link masses
    % link masses
    % m1 = 5; %0.1
    % m2 = 4;
    % m3 = 4;
    % m4 = 3;
    % m5 = 2.7;
    % m6 = 1.7;
    % m7 = 1.8;
    % m8 = 0.300000000000000;
    % m9 = 0;
    % 
    % % link inertias
    % I1 = [ 0.0745 0.1345 0.0800 0 0.0350 0]; %[Ixx Iyy Izz Iyz Ixz Ixy]. %[ 1 1 1 0 0 0]
    % I2 = [0.161200000000000,0.147600000000000,0.023600000000000,0.014400000000000,0,0];
    % I3 = [0.070980000000000,0.025056360000000,0.057924360000000,-0.009912000000000,-5.040000000000000e-05,-7.079999999999999e-05];
    % I4 = [0.133400000000000,0.125700000000000,0.012700000000000,-0.011700000000000,0,0];
    % I5 = [0.045241500000000,0.013121200000000,0.041120300000000,-0.006150600000000,0,0];
    % I6 = [0.030568900000000,0.027819217000000,0.005749717000000,-0.002713200000000,-1.292000000000000e-05,-3.570000000000000e-06];
    % I7 = [0.005000936000000,0.003600288000000,0.004700648000000,-4.320000000000000e-07,0,0];
    % I8 = [0.001120000000000,0.001120000000000,1.000000000000000e-03,0,0,0];
    % I9 = [0,0,0,0,0,0];
    % % COM
    % COM1 = [0.26 0 0.07]; % [x y z] in body frame
    % COM2 = [0 -0.0300 0.1200]; % [x y z] in body frame
    % COM3 = [3.000000000000000e-04,0.059000000000000,0.042000000000000]; % [x y z] in body frame
    % COM4 = [0,0.030000000000000,0.130000000000000]; % [x y z] in body frame
    % COM5 = [0,0.067000000000000,0.034000000000000]; % [x y z] in body frame
    % COM6 = [1.000000000000000e-04,0.021000000000000,0.076000000000000]; % [x y z] in body frame
    % COM7 = [0,6.000000000000000e-04,4.000000000000000e-04]; % [x y z] in body frame
    % COM8 = [0,0,0.020000000000000];
    % COM9 = [0,0,0];
    % % dh params
    % % [a alpha d theta]
    % % dhparams = [ 0 0 0 0;
    % %             l1 0 0 0;
    % %             l2 pi/2 0 pi;
    % %             l3 0 0 0;
    % %             l4 pi/2 0 pi;
    % %             l5 0  0  0];
    % 
    % 
    % dhparams = [ 0 0 0 0;
    %              0 0 0.36 0;
    %              0 pi/2 0 0;
    %              0 -pi/2 0.42 0;
    %              0 -pi/2 0 0;
    %              0  pi/2 0.40 0;
    %              0  pi/2  0  0;
    %              0 -pi/2 0 0
    %              0  0    0.126 0];
    
    
    % create robot
    robot = rigidBodyTree;
    
    % define bodies and Joints
    jnt1 = rigidBodyJoint('jnt1','fixed');
    body1 = rigidBody('link_0');
    body1.Mass = m1;
    body1.CenterOfMass = COM1;
    body1.Inertia = I1;
    
    jnt2 = rigidBodyJoint('jnt2','revolute');
    body2 = rigidBody('link_1');
    body2.Mass = m2;
    body2.CenterOfMass = COM2;
    body2.Inertia = I2;
    
    jnt3 = rigidBodyJoint('jnt3','revolute');
    body3 = rigidBody('link_2');
    body3.Mass = m3;
    body3.CenterOfMass = COM3;
    body3.Inertia = I3;
    
    jnt4 = rigidBodyJoint('jnt4','revolute');
    body4 = rigidBody('link_3');
    body4.Mass = m4;
    body4.CenterOfMass = COM4;
    body4.Inertia = I4;
    
    jnt5 = rigidBodyJoint('jnt5','revolute');
    body5 = rigidBody('link_4');
    body5.Mass = m5;
    body5.CenterOfMass = COM5;
    body5.Inertia = I5;
    
    jnt6= rigidBodyJoint('jnt6','revolute');
    body6 = rigidBody('link_5');
    body6.Mass = m6;
    body6.CenterOfMass = COM6;
    body6.Inertia = I6;
    
    jnt7 = rigidBodyJoint('jnt7','revolute');
    body7 = rigidBody('link_6');
    body7.Mass = m7;
    body7.CenterOfMass = COM7;
    body7.Inertia = I7;
    
    jnt8 = rigidBodyJoint('jnt8','revolute');
    body8 = rigidBody('link_7');
    body8.Mass = m8;
    body8.CenterOfMass = COM8;
    body8.Inertia = I8;
    
    endeff = rigidBody('link_ee');
    endeff.Mass = m9;
    endeff.CenterOfMass = COM9;
    endeff.Inertia = I9;
    
    
    
    % set frame transforms
    setFixedTransform(jnt1,dhparams(1,:),'mdh');
    setFixedTransform(jnt2,dhparams(2,:),'mdh');
    setFixedTransform(jnt3,dhparams(3,:),'mdh');
    setFixedTransform(jnt4,dhparams(4,:),'mdh');
    setFixedTransform(jnt5,dhparams(5,:),'mdh');
    setFixedTransform(jnt5,dhparams(6,:),'mdh');
    setFixedTransform(jnt6,dhparams(7,:),'mdh');
    setFixedTransform(jnt7,dhparams(8,:),'mdh');
    setFixedTransform(endeff.Joint,dhparams(9,:),'mdh');
    
    % Assign these Joints to the body
    body1.Joint = jnt1;
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    body6.Joint = jnt6;
    body7.Joint = jnt7;
    body8.Joint = jnt8;
    
    % Add these rigid bodies to the tree
    addBody(robot,body1,'base')
    addBody(robot,body2,'link_0')
    addBody(robot,body3,'link_1')
    addBody(robot,body4,'link_2')
    addBody(robot,body5,'link_3')
    addBody(robot,body6,'link_4')
    addBody(robot,body7,'link_5')
    addBody(robot,body8,'link_6')
    addBody(robot,endeff,'link_7')
    
    
    robot.DataFormat = 'column';


end