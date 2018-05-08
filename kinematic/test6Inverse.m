Link_length = [1 3 0 3 0 2];  % Jpint (3 and 4),(5 and 6) are coupled and jointed togeth
DH_table = [[0               pi/2       Link_length(1)        pi/2];
            [Link_length(2)     0                   0            0];
            [0              -pi/2                   0        -pi/2];
            [0               pi/2       Link_length(4)           0];
            [0              -pi/2                   0            0];
            [0                  0       Link_length(6)           0]];
        
initJointAngles = [0 0 0 0 0 0] * pi / 180;
Positions = ForwardKinematics(initJointAngles, DH_table, true);
DrawRobotManipulator(Link_length, Positions);
R06 = 