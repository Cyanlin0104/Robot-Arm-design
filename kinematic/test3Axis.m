% clean up
clc;
close all;
% Set link length for DH-table
Link_length = [2 2 2];
% Set DH-table
DH_table = [0                pi/2              Link_length(1)             pi/2;
            Link_length(2)   0                 0                           0;
            Link_length(3)   0                 0                           0];

% Set groundtruth rotation angle for each joint
tJointAngles = [ 30 50 90 ] * pi / 180 ;

% compute forward and record groundtruth
[tPos, tR] = ForwardKinematics(tJointAngles, DH_table, false);
tp = tPos(:,4); % target_point(end-effector position)

% compute backward and compare with groundtruth 
JointAngles = InversePosKinematics(Link_length, tp);

[Pos, R] = ForwardKinematics(JointAngles, DH_table, false);
p = Pos(:,4)
figure;title("Solution");
DrawRobotManipulator(Link_length, Pos);
figure;title("Groundtruth");
DrawRobotManipulator(Link_length, tPos);
if tp ~= p
    disp("target Positions error, please check InversePosKinematics")
    tp
    p
    return
end
if ~checkMatrix(Pos,tPos)
    disp("Positions error, please check InversePosKinematics")
    JointAngles
    tJointAngles
    Pos
    tPos  
    
end
if ~checkMatrix(R, tR)
    disp("RotationMatrix error, please check InversePosKinematics")
    R
    tR
    return
end
if ~checkMatrix(JointAngles, tJointAngles)
    disp("JointAngles different, please comfirm values");
    JointAngles
    tJointAngles
    return
end
disp("test PASS!!!");
DrawRobotManipulator(Link_length, Pos);