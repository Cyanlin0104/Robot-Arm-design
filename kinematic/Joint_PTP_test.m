function Trajectory_Trapezoidal_4()

%%
    clear;
    clc;
    close all;
    %% input condition
    Spd = 0.2;
    qi = [  0;     0;     0;     0;     0;     0; 0];                   % initial angle
    qf = [ 59;    50;   104;    29;   -104;   20;  36];                 % final angle
    V_Lim = [ 20;    25;    30;    15;    30;   20 ; 12]*360/60*Spd;    % limit velocity of each axis
    A_Lim = [ 20;  12.5;    60;   7.5;    60;   20 ; 20]*360/60*Spd;    % limit acc of each axis
    
    %% Parameter setting
    N  = size(qi, 1);   % num axis
    h  = abs(qf - qi);  % angle of rotation
    global tick;
    tick  = 0.05;       % sample time
    sync_flag = 0;      % enable or disable sync traj
    
    Vv = V_Lim;         % initialize command of velocity 
    Aa = A_Lim;         % initialize command of acc
    Ta = zeros(N, 1);   % acc time of each axis
    T  = zeros(N, 1);   % total time of motion of each axis
    
    
    
    %determine the main axis, compute it's T_max, Ta_max, velocity and acc
    for i = 1 : size(qi, 1)
        if( h(i) >= (Vv(i)^2 / Aa(i))) % satisfy the condition of trapezium velocity 
            % find T_max of each axis, Ta_max. velocity and acc
            Ta(i) = Vv(i) / Aa(i);
            T(i) = h(i) / Vv(i) + Ta(i);
            Vv(i) = h(i) / (T(i) - Ta(i));
            Aa(i) = h(i) / ((T(i) - Ta(i))*Ta(i));
            
            
        else  %
            % find T_max of each axis, Ta_max. velocity and acc
            Ta(i) = sqrt(h(i)/Aa(i));
            T(i) = 2*Ta(i);
            Vv(i) = h(i) / Ta(i);
            Aa(i) = h(i) / (Ta(i)^2);
        end
    end
    
      ti          = 0;            % starting time
     [T_Main, Idx] = max(T);        % total motion time and id of main axis
      tf          = T_Max;        % 
      Ta_Main      = Ta(Idx);     % 
      h_Main      = h(Idx);       % 
      A_Main      = Aa(Idx);      % 
      V_Main      = Vv(Idx);      % 

    
   %% update the velocity and acc of each axis
    for i = 1 : N
        % 
        Slave_New_V = h(i) / (T_Main - Ta_Main);
        Slave_New_A = h(i) / ((T_Main - Ta_Main) * Ta_Main);
        
        if (Slave_New_A > Av(i))% updated_acc > acc (Acc time of Main axis too long)
            % update T_Max and Ta_Max
            Ta_Max = (V_Main * h(i)) / (h_Main * Slave_New_A);
            T_Max  = (h(i)/ Slave_New_A * Ta_Main) + Ta_Max;
            
            % set the angle, velocity, acc to Main axis
            h_Main = h(i);
            V_Main = Vv(i);
            A_Main = Aa(i);
        
        elseif (Slave_New_A < Av(i))% 更新後的速度大於原本的速度 主軸加速時間過短(小於某一從軸)
            % 更新 T_Max 與 Ta_Max
            Ta_Max = (Slave_New_V * h_Main) / (h(i) * A_Main)
            T_Max  = 
            
            % 把該從軸的角度、速度與加速度 更新成主軸(主軸換人當)
            h_Main = 
            V_Main = 
            A_Main = 
        end
    end
   
    % --------------------------- %%%.*   and ./ 
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
