clc;
clear all;
clf;
%%               Õýß\„ÓŒW
JointAngle = [ 0 0 20 ] * pi / 180 ; 
dof = 3;
link_length = [2;3;3];
DH_table = [0                pi/2               link_length(1)             pi/2;
            link_length(2)   0                 0                           0;
            link_length(3)   0                 0                           0];
        
kinematics(dof, JointAngle,DH_table);
%%               Äæß\„ÓŒW
% X = 10;    Y = 50;    Z = 20;
% kinematics( X, Y, Z );
