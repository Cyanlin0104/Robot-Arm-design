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
v0=[0;0;0];
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
I4=diag([ 0, 0, 0]);

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
w1 = propagation_w_R(w0,t1_dot,R0_1')
w1_dot = propagation_w_dot_R(w0,w0_dot,t1_dot,t1_ddot,R0_1');

v1 = propagation_v_R(w0, v0, P0_1, R0_1')
a1 = propagation_v_dot_R(w0, w0_dot, a0, P0_1, R0_1');
ac1 = propagation_v_dot_R(w1, w1_dot, a1, P1_c1, eye(3,3));

Fint1 = m1*ac1;
Nint1 = I1*w1_dot+cross(w1,I1*w1);

% link 2 (R)
w2 = propagation_w_R(w1,t2_dot,R1_2')
w2_dot = propagation_w_dot_R(w1,w1_dot,t2_dot,t2_ddot,R1_2');

v2 = propagation_v_R(w1, v1, P1_2, R1_2')
a2 = propagation_v_dot_R(w1, w1_dot, a1, P1_2, R1_2');
ac2 = propagation_v_dot_R(w2, w2_dot, a2, P2_c2, eye(3,3));
Fint2 = m2*ac2;
Nint2 = I2*w2_dot+cross(w2,I2*w2);

% link 3 (P)
w3 = propagation_w_P(w2,R2_3')
w3_dot = propagation_w_dot_P(w2_dot,R2_3');

v3 = propagation_v_P(w2, v2, d3_dot, P2_3, R2_3')
a3 = propagation_v_dot_P(w2,w2_dot,a2,d3_dot,d3_ddot,P2_3, R2_3');
ac3 = propagation_v_dot_R(w3, w3_dot, a3, P3_c3, eye(3,3));

Fint3 = m3*ac3;
Nint3 = I3*w3_dot+cross(w3,I3*w3);

% link 4 (R)
w4 = propagation_w_R(w3,t4_dot,R3_4')
w4_dot = propagation_w_dot_R(w3,w3_dot,t4_dot,t4_ddot,R3_4');

v4 = propagation_v_R(w3, v3, P3_4, R3_4')
a4 = propagation_v_dot_R(w3, w3_dot, a3, P3_4, R3_4');
ac4 = propagation_v_dot_R(w4, w4_dot, a4, P4_c4, eye(3,3));

Fint4 = m4*ac4;
Nint4 = I4*w4_dot+cross(w4,I4*w4);

%% INWARD Propagation

f4 = [0;0;0] + Fint4;
n4 = back_propagation_moment([0 0 0]', [0 0 0]', Nint4, Fint4, [0 0 0]', P4_c4, eye(3,3));

f3 = R3_4*f4 + Fint3;
n3 = back_propagation_moment(n4, f4, Nint3, Fint3, P3_4, P3_c3, R3_4);

f2 = R2_3*f3+Fint2;
n2 = back_propagation_moment(n3, f3, Nint2, Fint2, P2_3, P2_c2, R2_3);

f1 = R1_2*f2+Fint1;
n1 = back_propagation_moment(n2, f2, Nint1, Fint1, P1_2, P1_c1, R1_2);

%% projection onto direction of revolutional axes of motor
tau1 = simplify(n1'*[0 0 1]')
tau2 = simplify(n2'*[0 0 1]')
tau3 = simplify(f3'*[0 0 1]')
tau4 = simplify(n4'*[0 0 1]')


MCG = [tau1; tau2; tau3; tau4];
simplify(MCG);
