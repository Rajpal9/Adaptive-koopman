function robot = robot_selector(robot_name, modularity)

    if strcmp(robot_name, '3R') && strcmp(modularity, '2D') == 1
        robot = robot_3R_2D;
    elseif strcmp(robot_name, '3R') && strcmp(modularity, '3D') == 1
        robot = robot_3R_3D;
    elseif strcmp(robot_name, '4R') && strcmp(modularity, '2D') == 1
        robot = robot_4R_2D;
    elseif strcmp(robot_name, '4R') && strcmp(modularity, '3D') == 1
        robot = robot_4R_3D;
    elseif strcmp(robot_name, '5R') && strcmp(modularity, '2D') == 1
        robot = robot_5R_2D;
    elseif strcmp(robot_name, '5R') && strcmp(modularity, '3D') == 1
        robot = robot_5R_3D;
    elseif strcmp(robot_name, '7R') && strcmp(modularity, '3D') == 1
        robot = robot_7R_3D;
    elseif strcmp(robot_name, 'Kuka') == 1
        robot = loadrobot('kukaIiwa14');
        robot.DataFormat = 'column';

    end
end
