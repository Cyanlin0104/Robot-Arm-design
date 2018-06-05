function Trajectory_Trapezoidal_4()

%%
    clear;
    clc;
    close all;
    %% input condition
    Spd = 0.2;
    qi = [  0;     0;     0;     0;     0;     0; 0];                   % initial angle
    qf = [ 59;    50;   104;    29;   -104;   20;  36];                 % final angle
    V_Lim = [ 20;    25;    30;    15;    30;   20 ; 12]*360/60*Spd;    % limit Velocity of each Joint 
    A_Lim = [ 20;  12.5;    60;   7.5;    60;   20 ; 20]*360/60*Spd;    % limit acceleration of each Joint
    
    
    %% Parameter setting
    N  = size(qi, 1);   % num_Axis
    h  = abs(qf - qi);  % angles need to rotate (in abs)
    global tick;
    tick  = 0.05;       % sample time
    sync_flag = 0;      % enable or disable sync traj
    
    Vv = V_Lim;         % 
    Aa = A_Lim;         % 
    Ta = zeros(N, 1);   % time of accelerating of each Joint
    T  = zeros(N, 1);   % time of motion of each Joint
    
    %% determine the main axis, figure out T_Max, Ta_Max, V, A
    for i = 1 : size(qi, 1)
        if()  % under Trapezoidal Velocity condition 
         
            
        else  %不滿足梯形速度條件
            % 計算各軸之T_Max,  Ta_Max, 速度和加速度 
        end
    end
    ti          = 0;            % 起始時間
    [T_Max Idx] = max(T);       % 主軸的移動時間與主軸ID
    tf          = T_Max;        % 主軸的終點時間
    Ta_Max      = Ta(Idx);      % 主軸的加速時間
    h_Main      = h(Idx);       % 主軸的轉動角度
    A_Main      = Aa(Idx);      % 主軸加速度
    V_Main      = Vv(Idx);      % 主軸速度

    
   %% 更新各軸之速度與加速度，並檢查是否超出自身極限
    for i = 1 : N
        % 各軸配合主軸(T_Max與Ta_Max)，更新其速度與加速度
        master_V = 
        master_A = 
        
        if % 更新後的加速度大於原本的加速度 (主軸加速時間過長(大於某一從軸))
            %更新 T_Max 與 Ta_Max
            Ta_Max = 
            T_Max  = 
            
            % 把該從軸的角度、速度與加速度 更新成主軸(主軸換人當)
            h_Main = 
            V_Main = 
            A_Main = 
        
        elseif % 更新後的速度大於原本的速度 主軸加速時間過短(小於某一從軸)
            % 更新 T_Max 與 Ta_Max
            Ta_Max = 
            T_Max  = 
            
            % 把該從軸的角度、速度與加速度 更新成主軸(主軸換人當)
            h_Main = 
            V_Main = 
            A_Main = 
        end
    end
   
    % ---------------------------
    Vv  = 
    Aa  = 
    Dis =
    % ---------------------------
    
    % ======= 判斷是否要開啟速度同步 =======
    if(sync_flag)
        % 令終點時間等於規劃好的最大時間
    else
        % 驅動速度等於最大速度
        % 驅動加速度等於最大加速度
    end
    % =====================================
    
    All_Joint_Q_traj = [];
    All_Joint_V_traj = [];
    All_Joint_A_traj = [];
    for i = 1 : size(qi, 1)
        % 計算Q V A在每一個sample time下的值
        if(sync_flag)
            [Q, V, A] = Trapezoidal_Curve(qi(i), qf(i), ti, T_Max, Ta_Max, Vv(i), Aa(i));
        else
            [Q, V, A] = Trapezoidal_Curve(???);
        end
        
             %% ---- 畫圖的部分 begin ---
        t  = ti : tick : tf;
        t  = [ti-0.01, ti, t, tf, tf+0.01];
        tmp_Q = [ 0, 0,  Q,  Q(end)*ones(1,(length(t)-length(Q)-2))  ];
        tmp_V = [ 0, 0,  V,  V(end)*ones(1,(length(t)-length(V)-2))  ];
        tmp_A = [ 0, 0,  A,  A(end)*zeros(1,(length(t)-length(A)-2)) ];
        
        All_Joint_Q_traj = [All_Joint_Q_traj ;tmp_Q];
        All_Joint_V_traj = [All_Joint_V_traj ;tmp_V];
        All_Joint_A_traj = [All_Joint_A_traj ;tmp_A];
        
    end
    PlotResult(1, t , All_Joint_Q_traj , 'Position', 'rad', '[s]');  
    PlotResult(2, t , All_Joint_V_traj , 'Velocity', 'deg/s', '[s]' );
    PlotResult(3, t , All_Joint_A_traj , 'Acceleration', 'deg/s^2', '[s]' );
    

function SaveData(filename, Data)
    fid=fopen(filename,'w');
    fprintf(fid,' %g ',Data);
    fclose(fid);  
 
function PlotResult(No, t, Data, Title, yLabel, xLabel )
    figure(No)
    plot(t , Data(1, 1:end), '-r');  hold on
    plot(t , Data(2, 1:end), '--g'); 
    plot(t , Data(3, 1:end), '.-b'); 
    plot(t , Data(4, 1:end), '+-c');  
    plot(t , Data(5, 1:end), ':m');  
    plot(t , Data(6, 1:end), 'x-k'); 
    plot(t , Data(7, 1:end), 'x-c'); 
    hold off
    legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7');
    title(Title), ylabel(yLabel), xlabel(xLabel)
    Q1_Max = max(max(Data)) + (max(max(Data)) - min(min(Data))) * 0.1;
    Q1_Min = min(min(Data)) - (max(max(Data)) - min(min(Data))) * 0.1;
    axis([min(t), max(t), Q1_Min, Q1_Max])    
        
function [QQ, VV, AA] = Trapezoidal_Curve(qi, qf, ti, tf, Ta, Vv, Aa)
    global tick;
    Vv = sign(qf - qi) * Vv;
    Aa = sign(qf - qi) * Aa;
    
    QQ = []; VV = []; AA = [];
    for t = ti : tick : tf
        % 避免小數點的精準度錯誤，而造成誤判
        % ex. 可能會發生 10.000 > 10 這種情況 
        if(roundn(, -4) < roundn(, -4))    % 在加速區間
            qt = 
            vt = 
            at = 
        elseif()                           % 在等速區間
            qt = 
            vt = 
            at = 
        else                               % 在減速區間
            qt = 
            vt = 
            at = 
        end
        QQ  = [QQ qt];
        VV  = [VV vt] ;
        AA  = [AA at] ;
    end
