function kinematics(dof, JointAngle, DH_table)
%% 【 Step １】 定義手臂自由度與DH連桿參數表與齊次轉換函數
        % 手臂的自由度

        % 手臂的各軸長度        
%% 【 Step 2 】 計算逆運動學
          %InverseKinematics() 
%% 【 Step 3 】 計算正運動學        
        % DH 參數表        -->  % 建立DHparameter( )          
        % 產生齊次轉換函數  -->   % 建立GenerateTransformationMatrices( )
%        ForwardKinematics(dof,JointAngle,Positions, DH_table)
%% 【 Step 4 】 繪製手臂
        pos = ForwardKinematics(dof, JointAngle, DH_table);
        DrawRobotManipulator(dof, pos)  
end

% =========================================================================
%                              【Functions】 
% =========================================================================
%%  DH 參數表
function  DH_table = DHparameter()
% 三軸的 DH 參數 [ a      α        d       θ ]
end
%%  齊次轉換矩陣
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
%% 正運動學
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
%% 逆運動學
function  InverseKinematics()
end   


%%  畫圖 畫機械手臂 輸入: 自由度，各關節的位置，各關節的座標
function  DrawRobotManipulator(dof, Positions)
hold on;
for i = 1 : dof 
    plotLP(Positions(:,i),Positions(:,i+1))
end
axis([-10 10 -10 10 -10 10])
grid on;
view(3);
end