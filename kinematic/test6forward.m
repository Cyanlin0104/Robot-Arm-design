[DH_table, Link_length] = get6axisParam();
prompt = 'Enter an array indicating Joint angles(in degree) >> ex: [90 60 30 30 50 30]: \n';
%f = figure('name','Manipulator simulation');
while(true)
inputs = input(prompt);
if isempty(inputs)
    break;
end
%clf;
JointAngles = inputs  * pi / 180;
[Positions, R] = ForwardKinematics(JointAngles, DH_table, true);
% given condition 
tR06 = R(:,:,7)
tP = Positions(:,7)
% groundTruth
tR03 = R(:,:,4)
tow = Positions(:,5)
tJointAngles = inputs
DrawRobotManipulator(Link_length, Positions);
end

