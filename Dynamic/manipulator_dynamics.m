clear;
clc;

syms d1  a1 a2 d2 d3 r1 r2 r3 l1 l2 l3 g m1 m2 m3 m4 M real;
syms t1 t1_dot t1_ddot t2 t2_dot t2_ddot t4 t4_dot t4_ddot d3_dot d3_ddot real;
syms Ixx_1 Iyy_1 Izz_1 Ixx_2 Iyy_2 Izz_2 Ixx_3 Iyy_3 Izz_3 Ixx_4 Iyy_4 Izz_4 real;


%initial conditions

DoF = 4;
w0=[0;0;0];
w0_dot=[0;0;0]; 
a0=[0;0;g];  
P0_1=[0;0;d1];
P1_c1=[r1*l1;0;0];
P1_2=[a1;0;d2];
P2_c2=[r2*l2;0;0];
P2_3=[a2;0;d3];
P3_c3=[0;0;r3*l3];
P3_4=[0;0;0];
P4_c4=[0;0;0];

I1=diag([ Ixx_1, Iyy_1, Izz_1]);
I2=diag([ Ixx_2, Iyy_2, Izz_2]);
I3=diag([ Ixx_3, Iyy_3, Izz_3]);
I4=diag([ Ixx_4, Iyy_4, Izz_4]);

%DH table
DH_crig = [ 0  0 d1 t1;  
      a1 0 d2 t2; 
      a2 0 d3  0; 
      0  0  0 t4];
T = sym('T', [4 4 DoF]);

for i=1:DoF
    a     = DH_crig(i,1);
    alpha = DH_crig(i,2);
    d     = DH_crig(i,3);
    theta = DH_crig(i,4);
    
    T(:,:,i) = simplify([cos(theta)                              -sin(theta)                  0                        a;
                         sin(theta)*cos(alpha)         cos(theta)*cos(alpha)        -sin(alpha)             d*sin(theta);
                         sin(theta)*sin(alpha)         cos(theta)*sin(alpha)         cos(alpha)                        d;
                                             0                             0                  0                        1]);

end

R0_1= T(1:3,1:3, 1);
R1_2= T(1:3,1:3, 2);
R2_3= T(1:3,1:3, 3);
R3_4= T(1:3,1:3, 4);

%% OUTWARD Propagation 
% link 1 (R)
w1 = ang_vel_revolute(w0,R0_1,t1_dot);
w1_dot = ang_acc_revolute([0;0;0],w0,R0_1,t1_dot,t1_ddot);

a1 = acc(a0,R0_1,w0,w0_dot,P0_1);
ac1 = acc_center(a1,w1,w1_dot,P1_c1);

Fint1 = m1*ac1;
Nint1 = I1*w1_dot+cross(w1,I1*w1);

% link 2 (R)
w2 = ang_vel_revolute(w1,R1_2,t2_dot);
w2_dot = ang_acc_revolute(w1_dot,w1,R1_2,t2_dot,t2_ddot);

a2 = acc(a1,R1_2,w1,w1_dot,P1_2);
ac2 = acc_center(a2,w2,w2_dot,P2_c2);

Fint2 = m2*ac2;
Nint2 = I2*w2_dot+cross(w2,I2*w2);

% link 3 (P)
w3 = ang_vel_pris(w2,R2_3,0);
w3_dot = ang_acc_pris(w2_dot,R2_3);

a3 = R2_3*acc_pris(a2,R2_3,w2,w2_dot,d3_dot,d3_ddot,P2_3,w3);
ac3 = acc_center(a3,w3,w3_dot,P3_c3);

Fint3 = m3*ac3;
Nint3 = I3*w3_dot+cross(w3,I3*w3);

% link 4 (R)
w4 = ang_vel_revolute(w3,R3_4,t4_dot);
w4_dot = ang_acc_revolute(w3_dot,w3,R3_4,t4_dot,t4_ddot);

a4 = acc(a3,R3_4,w3,w3_dot,P3_4);
ac4 = acc_center(a4,w4,w4_dot,P4_c4);

Fint4 = m4*ac4;
Nint4 = I4*w4_dot+cross(w4,I4*w4);

%% INWARD Propagation

f4 = R3_4*M*[0;0;-g]+Fint4;
n4 = moment_propagation(R3_4, [0;0;0], Nint4, Fint4, [0;0;0], P4_c4, M*[0;0;-g]);

f3 = R3_4*f4+Fint3;
n3 = moment_propagation(R2_3, n4, Nint3, Fint3, P3_4, P3_c3, f4);

f2 = R2_3*f3+Fint2;
n2 = moment_propagation(R1_2, n3, Nint2, Fint2, P2_3, P2_c2, f3);

f1 = R1_2*f2+Fint1;
n1 = moment_propagation(R0_1, n2, Nint1, Fint1, P1_2, P1_c1, f2);

%% projection onto direction of revolutional axes of motor
tau1 = n1'*[0 0 1]'
tau2 = n2'*[0 0 1]'
tau3 = f3'*[0 0 1]'
tau4 = n4'*[0 0 1]'


MCG = [tau1; tau2; tau3; tau4]
simplify(MCG)
