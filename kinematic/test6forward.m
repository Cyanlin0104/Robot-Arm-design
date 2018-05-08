Link_length = [1 3 0 3 0 2];  % Jpint (3 and 4),(5 and 6) are coupled and jointed togeth
DH_table = [[0               pi/2       Link_length(1)        pi/2];
            [Link_length(2)     0                   0            0];
            [0              -pi/2                   0        -pi/2];
            [0               pi/2       Link_length(4)           0];
            [0              -pi/2                   0            0];
            [0                  0       Link_length(6)           0]];
prompt = 'Enter an array indicating Joint angles(in degree) >> ex: [90 60 30 30 50 30]: \n';
f = figure('name','Manipulator simulation');
while(true)
inputs = input(prompt);
if isempty(inputs)
    break;
end
clf;
JointAngles = inputs  * pi / 180;
Positions = ForwardKinematics(JointAngles, DH_table, true);
DrawRobotManipulator(Link_length, Positions);
end

