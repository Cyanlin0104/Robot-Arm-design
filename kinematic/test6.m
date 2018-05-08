Link_length = [1 3 0 2 0 1];
DH_table = [[0               pi/2       Link_length(1)        pi/2];
            [Link_length(2)     0                   0            0];
            [0              -pi/2                   0        -pi/2];
            [0               pi/2       Link_length(4)           0];
            [0              -pi/2                   0            0];
            [0                  0       Link_length(6)           0]];
        
JointAngles = [ 0 30 -50 -30 -60 60] * pi / 180;
Positions = ForwardKinematics(JointAngles, DH_table);
DrawRobotManipulator(Link_length, Positions);

