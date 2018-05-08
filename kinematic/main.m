% clean up
clc;
clf;
% Set rotation angle for each joint
JointAngle = [ 90 30 -60 ] * pi / 180 ;
target_point = [0 2 5.5];
% Set link length for DH-table
link_length = [2 3 3];
% Set DH-table
DH_table = [0                pi/2              link_length(1)             pi/2;
            link_length(2)   0                 0                           0;
            link_length(3)   0                 0                           0];
        
kinematics_3(link_length, target_point, DH_table);