function kinematics(dof, JointAngle, DH_table)
%% �� Step ���� ���x�ֱ����ɶ��cDH�B�U�������c�R���D�Q����
        % �ֱ۵����ɶ�

        % �ֱ۵ĸ��S�L��        
%% �� Step 2 �� Ӌ�����\�ӌW
          %InverseKinematics() 
%% �� Step 3 �� Ӌ�����\�ӌW        
        % DH ������        -->  % ����DHparameter( )          
        % �a���R���D�Q����  -->   % ����GenerateTransformationMatrices( )
%        ForwardKinematics(dof,JointAngle,Positions, DH_table)
%% �� Step 4 �� �L�u�ֱ�
        pos = ForwardKinematics(dof, JointAngle, DH_table);
        DrawRobotManipulator(dof, pos)  
end

% =========================================================================
%                              ��Functions�� 
% =========================================================================
%%  DH ������
function  DH_table = DHparameter()
% ���S�� DH ���� [ a      ��        d       �� ]
end
%%  �R���D�Q���
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
%% ���\�ӌW
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
%% ���\�ӌW
function  InverseKinematics()
end   


%%  ���D ���Cе�ֱ� ݔ��: ���ɶȣ����P����λ�ã����P��������
function  DrawRobotManipulator(dof, Positions)
hold on;
for i = 1 : dof 
    plotLP(Positions(:,i),Positions(:,i+1))
end
axis([-10 10 -10 10 -10 10])
grid on;
view(3);
end