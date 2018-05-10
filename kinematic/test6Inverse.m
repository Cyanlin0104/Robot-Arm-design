Link_length = [1 3 0 3 0 2];  % Jpint (3 and 4),(5 and 6) are coupled and jointed togeth
DH_table = [[0               pi/2       Link_length(1)        pi/2];
            [Link_length(2)     0                   0            0];
            [0              -pi/2                   0        -pi/2];
            [0               pi/2       Link_length(4)           0];
            [0              -pi/2                   0            0];
            [0                  0       Link_length(6)           0]];
        

position_desired = [0 4 1];
orientation_desired = [50 40 90] .* pi / 180;

R06 = GenerateRotationMatrix(orientation_desired, "xyz")
wristLength = Link_length(6);
ow = decoupling(position_desired, R06, wristLength);

JointAngles0_3 = InversePosKinematics(Link_length, ow);

JointAngles = [JointAngles0_3 0 0 0]
[Positions, R06] = ForwardKinematics(JointAngles, DH_table, true);
DrawRobotManipulator(Link_length, Positions);