function Line_Traj_3_DOF()
    clear;clc;%clf;
    %% parameter for robot manipulator hardware
    DOF = 3; 
    Link            = [  10 30 30 ];    % length of links
    Parameter.Gear  = [  110 105 80 ];  % reduction ratio
  
    %% parameter for trajectory planning  
    Current.Pos = [ -20 10 0 ];         % initial pos
    Goal.Pos    = [  20 40 20 ];        % target pos
    
    Vel.Start   = 0                     % initial vel ( cm/s )
    Vel.Drv     = 50;                   % limit vel ( cm/s )
    Vel.Acc     = 100;                  % acceleration ( cm/s^2 )
    Vel.Tick    = 0.04;                % Sampling Time 
    max_mov_dis = Vel.Drv * Vel.Tick
    
    Error  = norm( Goal.Pos - Current.Pos );
    Vector = ( Goal.Pos - Current.Pos ) / Error;
    
   %% parameter for Finate State Machine(FSM) 
    Vel.State   = 'VEL_IDEL';   % initial state
    Vel.Now     = 0;            % current vel
    Vel.Dis     = 0;            % current distence moved
    %distance when max velocity is applied   50(cm/s)*0.05(s) = 2.5 cm/sample_time
    
     %% kinematics
    Current.Angle = IK(Current.Pos, Link);
    Info = FK(DOF, Current.Angle, Link);
    
    Goal.Angle = IK(Goal.Pos, Link);
    Info  = FK(DOF, Goal.Angle, Link);
    
    Path    = Current;
    %% for record
    Past_RPM = zeros(1,3);
    MotorACC = zeros(1,3);
    
    Rec_P   = Path.Pos';
    Rec_ErrP= Path.Pos;
    Rec_V   = 0;
    Rec_DD  = 0;
    Rec_Ang = Path.Angle';
    Rec_RPM = zeros(3,1);
    Rec_Acc = zeros(3,1);
    
    Ta = Vel.Drv / Vel.Acc;
    if(Ta*Vel.Drv >= abs(Error))
        Vel.Drv = sqrt(abs(Error)*Vel.Acc);
    end
    %% Process
    while( abs(Error) > 0.0001 )


        % get feedback and compute position 
        Feedback_Ang = Path.Angle;                      
        Info  = FK(DOF, Feedback_Ang, Link);   
        
        DrawRobotManipulator( DOF, Info.JointPos, Info.JointDir, 1 , [-145 30 ] );
        title('RRR Manipulator');
        
        % update vel and dis based on current vel and error (1/2))
        Vel = CalculateMovementDistance( Vel, Error );
        
        % update position on the path(2/2))
        Path.Pos   = Path.Pos + Vector .* Vel.Dis
        
        % update angular velocity of Joints
        Path.Angle = IK(Path.Pos, Link);
        Ang_V  = Path.Angle / Vel.Tick
        
        % angular velocity to RPM
        RPM    = (Ang_V*60)/(2*pi)
        
        % update the error from target position
        Error  = norm( Goal.Pos - Path.Pos );
        Vel.State
        pause( Vel.Tick );
        
        % record1
        MotorACC = ( Ang_V .* Parameter.Gear - Past_RPM ) ./ Vel.Tick ;
        Past_RPM =   Ang_V .* Parameter.Gear;
        Rec_P   = [ Rec_P   Info.Pos ];
        Rec_V    = [ Rec_V    Vel.Now ];
        Rec_Ang = [ Rec_Ang  Path.Angle'];
        Rec_RPM = [ Rec_RPM  RPM' ] ;
        Rec_Acc = [ Rec_Acc  MotorACC' ];
    end
    
    % record2
    Rec_P   = [ Rec_P   Path.Pos' ];
    Rec_Ang = [ Rec_Ang  Path.Angle'];
    
    Rec_V   = [ Rec_V    0 ];
    Rec_RPM = [ Rec_RPM  zeros(3,1) ] ;
    Rec_Acc = [ Rec_Acc  zeros(3,1) ];
    
    %% Draw Trajectory
    figure(1)
    plot3(  Rec_P(1,:), Rec_P(2,:), Rec_P(3,:) ,'bo');
    
    t  =  ( 1:size(Rec_V,2) ) * Vel.Tick;
    figure(2)
    plot(  t , Rec_V ,'-bo');
    axis( [ min(t) max(t) min(Rec_V) max(Rec_V) + 2] )
    
    
     DrawAngle( t , Rec_Ang );
     DrawRPM( t , Rec_RPM );
     DrawACC( t, Rec_Acc );
 

end

%% FSM
function Vel = CalculateMovementDistance( Vel , Error )
    Vel.Dis = 0;
    
    if( strcmp( Vel.State, 'VEL_IDEL' ) )
         Vel.Now   =  0; 
         Vel.State = 'VEL_ACC';
         Vel.Dis = Vel.Now * Vel.Tick;
     
    elseif( strcmp( Vel.State, 'VEL_ACC' ) )
        Vel.Dis = Vel.Now * Vel.Tick + (Vel.Acc*Vel.Tick^2)/2
        Vel.Now = Vel.Now + Vel.Acc * Vel.Tick;
    
        if(Vel.Now > Vel.Drv)
            Vel.State = 'VEL_KEEP';
            Vel.Now = Vel.Drv;
        else
            Vel.State = 'VEL_ACC';
        end
     
    elseif( strcmp( Vel.State, 'VEL_KEEP' ) )
        [IsDec, Vel] = IsDeceleration( Vel, Error );
        
        if(IsDec)
            Vel.State = 'VEL_DEC';
        else
            Vel.State = 'VEL_KEEP';
            Vel.Now = Vel.Drv;
            Vel.Dis = Vel.Now*Vel.Tick;
        end
        
    elseif( strcmp( Vel.State, 'VEL_DEC' ) )
		Vel_Next = Vel.Now - Vel.Acc*Vel.Tick;

        Vel.Now = Vel_Next;
        if(Vel.Now < 0)
            Vel.Now = 0;
            Vel.State = 'VEL_IDEL';
        else
            Vel.State = 'VEL_DEC';
        end
            Vel.Dis = Vel.Now*Vel.Tick + (Vel.Acc*Vel.Tick^2)/2;
    else
        fprintf('err')
    end

	if( Vel.Dis >= abs(Error) )
        Vel.Dis = abs(Error);
    end
    
	if ( Error < 0 )
		Vel.Dis =  -1.0 * Vel.Dis;
    end
end
% trapizoid velocity
function [IsDec, Vel] = IsDeceleration( Vel , Error)
    % time needed for Vel.Now to stop
	t_dec  = Vel.Drv / Vel.Acc; 

    % decceleration distance  
	d1 = Vel.Drv * Vel.Tick;
    d2 = Vel.Drv * t_dec / 2;
    
   
    % if Current Distance >= Decceleration Distance
    % we should deccelerate
	if(abs(Error) <= d1+d2)
        t1      = (abs(Error-d2)) / Vel.Drv;
        t2      = Vel.Tick - t1;
        
        Vel_next = Vel.Now - Vel.Acc*t2;
        Vel.Dis = Vel_next * Vel.Tick + (Vel.Now - Vel_next)*Vel.Tick / 2;
        Vel.Now = Vel_next;
		IsDec = 1;
	else
		IsDec = 0;
    
   
    end
end
%% ���B�ʾ�FK
function Info = FK(DOF, Angle, Link)
    %  �׫׶q�� �y�׶q
    Angle = Angle*pi/180;
    
    % ================ Step1 �t�m���u�y�Шt ================  
    % (1) �M�w������u�U���`��0�ת����A
    % (2) �M�w�U���`�y�Шt��Z�b
    % (3) �w�q�U���`��XY�b 
    % (4) �̫�Хܥ����I���y�Шta,s,n
      
    % ================ Step2 �إߢҢְѼƳs��� ================  
    % �@��ai�B�\i�Bdi�B�ci�|�ӰѼ�
    % ��i : distance(zi-1, zi) along xi
    % �\i : angle(zi-1, zi) about xi
    % ��i : distance(oi-1,xi) along zi-1
    % �ci : angle(xi-1, xi) about zi
    %
    %                   �Ң� Parameter Table
    %+---+-----------+-----------+-----------+-----------+
    %| j |        �� |     alpha |        �� |     theta |
    %+---+-----------+-----------+-----------+-----------+
    %| 1 |          0|         90|        ��1|         90|
    %| 2 |        ��1|          0|          0|          0|
    %| 3 |        ��2|          0|          0|          0|
    %+---+-----------+-----------+-----------+-----------+
    %
    % RRR��DH�s��Ѽ� [  ��      �\    �@��      �@�c   ]
    DHParameter  =  [   0     pi/2   Link(1)     pi/2;      
                      Link(2)      0      0        0;       
                      Link(3)      0      0        0   ];   
                    
    %================ Step3 �p��Ңֻ����ഫ�x�} ================   
    % ��l�]�w
    JointPos = [0 ;0 ;0];
    JointDir = [1 0 0; 0 1 0; 0 0 1];
    T0_3 = [ 1 0 0 0; 0 1 0 0;0 0 1 0; 0 0 0 1];    
    
    % �p��A1, A2, A3
    for i = 1 : DOF
        A =  GenerateTransformationMatrices( Angle(i), DHParameter(i,:));
        T0_3 = T0_3 * A;
        JointPos = [ JointPos T0_3( 1:3, 4)];    % �x�s���`���y�Ц�m�A�x�s�����I����m O0_6 (O0_6��1*3�x�}) 
        JointDir = [ JointDir T0_3( 1:3, 1:3 ) ];
    end
    
    Info.Pos = JointPos(1:3,4);
    Info.JointPos = JointPos;
    Info.JointDir = JointDir;
    
end

%% �Ңֻ����ഫ�x�}
function A =  GenerateTransformationMatrices( Theta, DH_Parameter ) 
    %   �̧ǥN�J �ثe�U�b������, �̧ǥN�J �U�C(�b)���s��Ѽƪ�  Ex:�Ĥ@�b, �Ĥ@�C
    %   �̧ǭp�� T0_1~T5_6 = A1~A6 
    %   DH_Parameter (1) = ��
    %   DH_Parameter (2) = �\      
    %   DH_Parameter (3) = ��       
    %   DH_Parameter (4) = �c
                                                                                                       
    C_Theta = cos( Theta + DH_Parameter(4) );
    S_Theta = sin( Theta + DH_Parameter(4) );
    C_Alpha = cos( DH_Parameter(2) );
    S_Alpha = sin( DH_Parameter(2) ); 
    
    A = [   C_Theta   -1*S_Theta*C_Alpha        S_Theta*S_Alpha     DH_Parameter(1) * C_Theta; 
            S_Theta      C_Theta*C_Alpha     -1*C_Theta*S_Alpha     DH_Parameter(1) * S_Theta;
            0                    S_Alpha                C_Alpha                DH_Parameter(3);     %%%%%%%%%%%
            0                          0                      0                             1   ];
end

%% �f�B�ʾ�IK
function Angle = IK(Goal, Link)

    % �Ĥ@�b���F���� theta1
    Angle(1) = -atan2(Goal(1), Goal(2)); 
  
    % �ĤT�b���F���� theta3
    cosTheta3     = ((Goal(3)- Link(1))^2  +  Goal(1)^2  +  Goal(2)^2 - (Link(2)^2+Link(3)^2)) / (2.0 *Link(2)*Link(3));
    c3 = cosTheta3;
    Angle(3) = atan2( -1*sqrt(1-c3^2),c3 );
     
    % �ĤG�b���F���� theta2
    r = sqrt(Goal(1)^2+Goal(2)^2);
    Theta3 =Angle(3);
    Angle(2) = atan2(Goal(3)-Link(1),r)-...
    atan2(Link(3)*sin(Theta3), Link(2)+Link(3)*cos(Theta3));   

     % �y�׶q �� �׫׶q
    Angle = Angle*180/pi;
end

%%  �e�� �e������u ��J: �ۥѫסA�U���`����m�A�U���`���y��
function DrawRobotManipulator( DOF, JointPos, JointDir , FigNum , View )

    figure(FigNum)

    hold off
    plot3( JointPos(1,1:end), JointPos(2,1:end), JointPos(3,1:end),'linewidth', 4)
    hold on
    plot3( JointPos(1,1:end), JointPos(2,1:end), JointPos(3,1:end),'ro','linewidth', 7);

    grid on
    xlabel('X�b');
    ylabel('Y�b');
    zlabel('Z�b');
    X =10;
    Y =10;
    Z =20;
    %-------------------------------------
    BaseX = [X X -X -X X];
    BaseY = [Y -Y -Y Y Y];
    BaseZ = [0 0 0 0 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [X X X X X];
    BaseY = [Y -Y -Y Y Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [X -X -X X X];
    BaseY = [Y Y Y Y Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [-X -X -X -X -X];
    BaseY = [-Y Y Y -Y -Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')
    %-------------------------------------
    BaseX = [X -X -X X X];
    BaseY = [-Y -Y -Y -Y -Y];
    BaseZ = [0 0 -Z -Z 0];
    patch(BaseX,BaseY,BaseZ,'k')

    for i = 1 : DOF+1

        %-----------------------------------------
        nUnit_v = JointPos( : , i ) + 5 * JointDir( : , 3 * (i-1) + 1 );        % normal ����V�q 
        nBase   = [JointPos( : , i ) nUnit_v];
        plot3(nBase(1,1:end),nBase(2,1:end),nBase(3,1:end),'y','linewidth',2);  
        %--------------------------------------------
        sUnit_v = JointPos( : , i ) + 5 * JointDir( : , 3 * (i-1) + 2 );        % sliding �ưʦV�q
        sBase   = [JointPos( : , i ) sUnit_v];
        plot3(sBase(1,1:end),sBase(2,1:end),sBase(3,1:end),'g','linewidth',2);
        %--------------------------------------------
        aUnit_v = JointPos( : , i ) + 5 * JointDir( : , 3 * (i-1) + 3 );        % approach �a��
        aBase   = [JointPos( : , i ) aUnit_v];
        plot3(aBase(1,1:end),aBase(2,1:end),aBase(3,1:end),'r','linewidth',2);
        %--------------------------------------------
        hold on
    end
    view( View )
    %axis equal
    axis( [-50 50 -30 80 -20 80])
end

%%
function DrawAngle( t, Rec_Ang )
figure(3)
subplot(3,1,1)
plot( t, Rec_Ang(1,:));
title('\theta1'); xlabel('Sec'); ylabel('degree'); grid on
subplot(3,1,2)
plot(t, Rec_Ang(2,:));
title('\theta2'); xlabel('Sec'); ylabel('degree'); grid on
subplot(3,1,3)
plot(t, Rec_Ang(3,:));
title('\theta3'); xlabel('Sec'); ylabel('degree'); grid on

end

function DrawRPM( t, Rec_RPM )
figure(4)
subplot(3,1,1)
plot(t, Rec_RPM(1,:),'-bo');
title('\theta1'); xlabel('Sec'); ylabel('RPM'); grid on
subplot(3,1,2)
plot(t, Rec_RPM(2,:),'-bo');
title('\theta2'); xlabel('Sec'); ylabel('RPM'); grid on
subplot(3,1,3)
plot(t, Rec_RPM(3,:),'-bo');
title('\theta3'); xlabel('Sec'); ylabel('RPM'); grid on

end

function DrawACC( t, Rec_ACC )
figure(5)
subplot(3,1,1)
plot(t, Rec_ACC(1,:) ,'-bo');
title('\theta1'); xlabel('Sec'); ylabel('Acc'); grid on
subplot(3,1,2)
plot(t, Rec_ACC(2,:) ,'-bo');
title('\theta2'); xlabel('Sec'); ylabel('Acc'); grid on
subplot(3,1,3)
plot(t, Rec_ACC(3,:) ,'-bo');
title('\theta3'); xlabel('Sec'); ylabel('Acc'); grid on

end
