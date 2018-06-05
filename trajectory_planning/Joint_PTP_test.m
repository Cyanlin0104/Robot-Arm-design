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
         
            
        else  %��������γt�ױ���
            % �p��U�b��T_Max,  Ta_Max, �t�שM�[�t�� 
        end
    end
    ti          = 0;            % �_�l�ɶ�
    [T_Max Idx] = max(T);       % �D�b�����ʮɶ��P�D�bID
    tf          = T_Max;        % �D�b�����I�ɶ�
    Ta_Max      = Ta(Idx);      % �D�b���[�t�ɶ�
    h_Main      = h(Idx);       % �D�b����ʨ���
    A_Main      = Aa(Idx);      % �D�b�[�t��
    V_Main      = Vv(Idx);      % �D�b�t��

    
   %% ��s�U�b���t�׻P�[�t�סA���ˬd�O�_�W�X�ۨ�����
    for i = 1 : N
        % �U�b�t�X�D�b(T_Max�PTa_Max)�A��s��t�׻P�[�t��
        master_V = 
        master_A = 
        
        if % ��s�᪺�[�t�פj��쥻���[�t�� (�D�b�[�t�ɶ��L��(�j��Y�@�q�b))
            %��s T_Max �P Ta_Max
            Ta_Max = 
            T_Max  = 
            
            % ��ӱq�b�����סB�t�׻P�[�t�� ��s���D�b(�D�b���H��)
            h_Main = 
            V_Main = 
            A_Main = 
        
        elseif % ��s�᪺�t�פj��쥻���t�� �D�b�[�t�ɶ��L�u(�p��Y�@�q�b)
            % ��s T_Max �P Ta_Max
            Ta_Max = 
            T_Max  = 
            
            % ��ӱq�b�����סB�t�׻P�[�t�� ��s���D�b(�D�b���H��)
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
    
    % ======= �P�_�O�_�n�}�ҳt�צP�B =======
    if(sync_flag)
        % �O���I�ɶ�����W���n���̤j�ɶ�
    else
        % �X�ʳt�׵���̤j�t��
        % �X�ʥ[�t�׵���̤j�[�t��
    end
    % =====================================
    
    All_Joint_Q_traj = [];
    All_Joint_V_traj = [];
    All_Joint_A_traj = [];
    for i = 1 : size(qi, 1)
        % �p��Q V A�b�C�@��sample time�U����
        if(sync_flag)
            [Q, V, A] = Trapezoidal_Curve(qi(i), qf(i), ti, T_Max, Ta_Max, Vv(i), Aa(i));
        else
            [Q, V, A] = Trapezoidal_Curve(???);
        end
        
             %% ---- �e�Ϫ����� begin ---
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
        % �קK�p���I����ǫ׿��~�A�ӳy���~�P
        % ex. �i��|�o�� 10.000 > 10 �o�ر��p 
        if(roundn(, -4) < roundn(, -4))    % �b�[�t�϶�
            qt = 
            vt = 
            at = 
        elseif()                           % �b���t�϶�
            qt = 
            vt = 
            at = 
        else                               % �b��t�϶�
            qt = 
            vt = 
            at = 
        end
        QQ  = [QQ qt];
        VV  = [VV vt] ;
        AA  = [AA at] ;
    end
