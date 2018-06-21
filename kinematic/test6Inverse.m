[DH_table, Link_length] = get6axisParam();
% position and orientation desired.
position_desired = tP';
orientation_desired = [0 0 0] .* pi / 180;

% decouple and find position of wrist.
%R06 = GenerateRotationMatrix(orientation_desired, "xyz")

R06 = tR06;
wristLength = Link_length(6);
ow = decoupling(position_desired, R06, wristLength);
if ~checkMatrix(ow, tow)
disp('ow error');
end
ow
tow

JointAngles1_3 = InversePosKinematics(Link_length, ow);
if ~checkMatrix(JointAngles1_3, tJointAngles(1:3))
disp('JointAngle1_3 error')
end
JointAngles1_3 .* 180 / pi
tJointAngles(1:3)

JointAngles = [JointAngles1_3 0 0 0];
[Positions, R] = ForwardKinematics(JointAngles, DH_table, false);
R03 = R(:,:,4);
R30 = R03';
if ~checkMatrix(R03, tR03)
disp('R03 error');
end
R03
tR03

R36 = R30 * R06;

theta4 = atan2(R36(2,3),R36(1,3));
theta5 = -acos(R36(3,3));
theta6 = atan2(R36(3,2),-R36(3,1));

JointAngles = [JointAngles1_3 theta4 theta5 theta6];
[PositionsUpdate4_6, R] = ForwardKinematics(JointAngles, DH_table, true);
JointAngles = JointAngles .* 180 /pi;
<<<<<<< HEAD
if checkMatrix(ow, tow')
    disp('ow error');
    ow
    tow'
end
if checkMatrix(R03, tR03)
    disp(' R03 wrong');
    R03
    tR03
    return
end
if checkMatrix(R36, tR36)
    disp(' R36 wrong');
    R36
    tR36
    return
end
disp('test PASS')

=======
if ~checkMatrix(JointAngles, tJointAngles)
disp('JointAngles error');
end
JointAngles
tJointAngles
>>>>>>> 0d5ffcecf53822001d6e92aa028487a80bdc7092
DrawRobotManipulator(Link_length, PositionsUpdate4_6);

