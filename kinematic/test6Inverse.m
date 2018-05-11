Link_length = [1 3 0 3 0 2];  % Jpint (3 and 4),(5 and 6) are coupled and jointed togeth
DH_table = [[0               pi/2       Link_length(1)        pi/2];
            [Link_length(2)     0                   0            0];
            [0              -pi/2                   0        -pi/2];
            [0               pi/2       Link_length(4)           0];
            [0              -pi/2                   0            0];
            [0                  0       Link_length(6)           0]];
% position and orientation desired.
position_desired = tP';
orientation_desired = [0 0 0] .* pi / 180;

% decouple and find position of wrist.
%R06 = GenerateRotationMatrix(orientation_desired, "xyz")

R06 = tR06;
wristLength = Link_length(6);
ow = decoupling(position_desired, R06, wristLength);

JointAngles0_3 = InversePosKinematics(Link_length, ow);

JointAngles = [JointAngles0_3 0 0 0];
[Positions, R] = ForwardKinematics(JointAngles, DH_table, false);
R03 = R(:,:,4);
R30 = R03';

R36 = R30 * R06;
theta4 = atan2(R36(2,3),R36(1,3));
theta5 = acos(R36(3,3));
theta6 = atan2(R36(3,2),-R36(3,1));
JointAngles = [JointAngles0_3 theta4 theta5 theta6];
[PositionsUpdate4_6, R] = ForwardKinematics(JointAngles, DH_table, false);
JointAngles = JointAngles .* 180 /pi;
if ow ~= tow'
    disp(' ow !wrong!');
end
if (sum(sum(R03 ~= tR03,1),2) ~= 0)
    disp(' R03 wrong');
end
if (sum(sum(R03 ~= tR03,1),2) ~= 0)
    disp(' R36 wrong');
end
%DrawRobotManipulator(Link_length, PositionsUpdate4_6);

