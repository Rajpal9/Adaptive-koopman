function robot = robot_5R_3D
    % parameter
    % link length
    l1 = 0.33; % l1 = 0.33
    l2 = 0.33;
    l3 = 0.33;
    l4 = 0.33; % l1 = 0.33
    l5 = 0.33;
    
    % link masses
    % link masses
    m1 = 0.1; %0.1
    m2 = 0.1;
    m3 = 0.1;
    m4 = 0.1;
    m5 = 0.1;
    
    % link inertias
    I1 = [ 1 1 1 0 0 0]; %[Ixx Iyy Izz Iyz Ixz Ixy]. %[ 1 1 1 0 0 0]
    I2 = [ 1 1 1 0 0 0];
    I3 = [ 1 1 1 0 0 0];
    I4 = [ 1 1 1 0 0 0];
    I5 = [ 1 1 1 0 0 0];
    
    
    % COM
    COM1 = [l1/2 0 0]; % [x y z] in body frame
    COM2 = [l2/2 0 0]; % [x y z] in body frame
    COM3 = [l3/2 0 0]; % [x y z] in body frame
    COM4 = [l4/2 0 0]; % [x y z] in body frame
    COM5 = [l5/2 0 0]; % [x y z] in body frame
    
    % dh params
    % [a alpha d theta]
    dhparams = [ 0 0 0 0;
                l1 0 0 0;
                l2 pi/2 0 pi;
                l3 0 0 0;
                l4 pi/2 0 pi;
                l5 0  0  0];
                
    
    % create robot
    robot = rigidBodyTree;
    
    % define bodies and Joints
    jnt1 = rigidBodyJoint('jnt1','revolute');
    body1 = rigidBody('link_1');
    body1.Mass = m1;
    body1.CenterOfMass = COM1;
    body1.Inertia = I1;
    
    jnt2 = rigidBodyJoint('jnt2','revolute');
    body2 = rigidBody('link_2');
    body2.Mass = m2;
    body2.CenterOfMass = COM2;
    body2.Inertia = I2;
    
    jnt3 = rigidBodyJoint('jnt3','revolute');
    body3 = rigidBody('link_3');
    body3.Mass = m3;
    body3.CenterOfMass = COM3;
    body2.Inertia = I3;
    
    jnt4 = rigidBodyJoint('jnt4','revolute');
    body4 = rigidBody('link_4');
    body4.Mass = m4;
    body4.CenterOfMass = COM4;
    body4.Inertia = I4;
    
    jnt5 = rigidBodyJoint('jnt5','revolute');
    body5 = rigidBody('link_5');
    body5.Mass = m5;
    body5.CenterOfMass = COM5;
    body5.Inertia = I5;
    
    endeff = rigidBody('link_ee');
    endeff.Mass = 0;
    endeff.CenterOfMass = 0*COM3;
    endeff.Inertia = 0*I3;
    
    
    
    % set frame transforms
    setFixedTransform(jnt1,dhparams(1,:),'mdh');
    setFixedTransform(jnt2,dhparams(2,:),'mdh');
    setFixedTransform(jnt3,dhparams(3,:),'mdh');
    setFixedTransform(jnt4,dhparams(4,:),'mdh');
    setFixedTransform(jnt5,dhparams(5,:),'mdh');
    setFixedTransform(endeff.Joint,dhparams(6,:),'mdh');
    
    % Assign these Joints to the body
    body1.Joint = jnt1;
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    
    % Add these rigid bodies to the tree
    addBody(robot,body1,'base')
    addBody(robot,body2,'link_1')
    addBody(robot,body3,'link_2')
    addBody(robot,body4,'link_3')
    addBody(robot,body5,'link_4')
    addBody(robot,endeff,'link_5')
    
    robot.DataFormat = 'column';
end