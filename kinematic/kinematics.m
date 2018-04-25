function kinematics(link_length, dof, target_point, DH_table)
        JointAngle = InverseKinematics(link_length,target_point)
        pos = ForwardKinematics(dof, JointAngle, DH_table);
        DrawRobotManipulator(link_length, dof, pos)  
end

function  [A] = GenerateTransformationMatrices(theta, DH_params)
    a_dh = DH_params(1);
    d_dh = DH_params(3);
    arpha_dh = DH_params(2);    
    theta_dh = DH_params(4);
    carpha = cos(arpha_dh);
    sarpha = sin(arpha_dh);
    ctheta = cos(theta_dh + theta);
    stheta = sin(theta_dh + theta);
    
    A =    [ctheta   -stheta*carpha  stheta*sarpha    a_dh*ctheta;
            stheta   ctheta*carpha   -ctheta*sarpha   a_dh*stheta;
            0        sarpha          carpha           d_dh       ;      
            0        0               0                1          ];
    
end

function  [Pos] = ForwardKinematics(dof, JointAngle, DH_table)
    T = eye(4);
    Pos = zeros(3, dof + 1);
    Pos(:,1) = [0;0;0];
    for i = 1 : dof
        jointAngle = JointAngle(i);
        DH_params = DH_table(i, :);
        A = GenerateTransformationMatrices(jointAngle, DH_params);
        T = T * A;
        Pos(:, i+1) = T(1:3, 4)
    end

end

function  JointAngle = InverseKinematics(link_length, target_point)
xc = target_point(1)
yc = target_point(2)
zc = target_point(3)
L1 = link_length(1)
L2 = link_length(2)
L3 = link_length(3)
theta1 = atan2(yc,xc)
cthetaD = (L2^2 + L3^2 - (xc^2 + yc^2) - (zc - L1)^2) / (2*L2*L3)
theta3 =  acos(cthetaD) - pi;
theta2 = atan2(zc-L1,sqrt(xc^2 + yc^2)) - atan2(L3*sin(theta3),L2+L3*cos(theta3));
JointAngle = [theta1 theta2 theta3]
end   



function  DrawRobotManipulator(link_length, dof, Positions)
hold on;
for i = 1 : dof 
    plotLP(Positions(:,i),Positions(:,i+1))
end
% Base table
v = (sum(link_length)/length(link_length)).* 0.5 .* ...
    [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 0 ;1 -1 0; 1 1 0; -1 1 0];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', v, 'Faces', f, 'FaceColor', 'black');axis equal;

axis([-10 10 -10 10 -10 10])
grid on;
view(3);
end