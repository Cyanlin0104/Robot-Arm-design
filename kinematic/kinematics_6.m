function kinematics6()
link_length = [2 3 0 3 0 2];  % Jpint (3 and 4),(5 and 6) are coupled and jointed together.
dof = 6;
JointAngle = [ 0 30 -50 -30 -60 60] * pi / 180;


DH_table = [[0               pi/2       link_length(1)        pi/2];
            [link_length(2)     0                   0            0];
            [0              -pi/2                   0        -pi/2];
            [0               pi/2       link_length(4)           0];
            [0              -pi/2                   0            0];
            [0                  0       link_length(6)           0]];
        
Positions = ForwardKinematics(dof,JointAngle, DH_table);
DrawRobotManipulator(link_length, dof, Positions);
end
