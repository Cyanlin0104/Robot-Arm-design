clear;
clc;

syms d1  a1 a2 d2  r1 r2 r3 l1 l2 l3 g m1 m2 m3 m4 M real;
syms Ixx_1 Iyy_1 Izz_1 Ixx_2 Iyy_2 Izz_2 Ixx_3 Iyy_3 Izz_3 Ixx_4 Iyy_4 Izz_4 g real;
syms t real;
syms t1(t) t2(t) d3(t) t4(t);
syms t1_ t2_ d3_ t4_ real;
syms t1_dot_ t2_dot_ d3_dot_ t4_dot_ real;
syms t1_ddot_ t2_ddot_ d3_ddot_ t4_ddot_ real;
t1_dot = diff(t1(t),t);
t2_dot = diff(t2(t),t);
d3_dot = diff(d3(t),t);
t4_dot = diff(t4(t),t);

%initial conditions
DoF = 4;
w0=[0;0;0];
w0_dot=[0;0;0]; 
a0=[0;0;g];
v0=[0;0;0];
P0_1=[0;0;d1];
P1_c1=[r1;0;0];
P1_2=[a1;0;d2];
P2_c2=[r2;0;0];
P2_3=[a2;0;d3];
P3_c3=[0;0;r3];
P3_4=[0;0;0];
P4_c4=[0;0;0];

I1=diag([ Ixx_1, Iyy_1, Izz_1]);
I2=diag([ Ixx_2, Iyy_2, Izz_2]);
I3=diag([ Ixx_3, Iyy_3, Izz_3]);
I4=diag([ 0, 0, 0]);

%DH table
DH_crig = [ 0  0 d1 t1(t);  
            a1 0 d2 t2(t); 
            a2 0 d3(t)  0; 
            0  0  0 t4(t)];
        
Transformation = sym('T', [4 4 DoF]);

for i=1:DoF
    a     = DH_crig(i,1);
    alpha = DH_crig(i,2);
    d     = DH_crig(i,3);
    theta = DH_crig(i,4);
    
    Transformation(:,:,i) = simplify([cos(theta)                              -sin(theta)                  0                        a;
                         sin(theta)*cos(alpha)         cos(theta)*cos(alpha)        -sin(alpha)             d*sin(theta);
                         sin(theta)*sin(alpha)         cos(theta)*sin(alpha)         cos(alpha)                        d;
                                             0                             0                  0                        1]);

end

R0_1= Transformation(1:3,1:3, 1);
R1_2= Transformation(1:3,1:3, 2);
R2_3= Transformation(1:3,1:3, 3);
R3_4= Transformation(1:3,1:3, 4);
T0_1 = Transformation(:,:,1);
T1_2 = Transformation(:,:,2);
T2_3 = Transformation(:,:,3);
T3_4 = Transformation(:,:,4);

%% OUTWARD Propagation 
% link 1 (R)
w1 = propagation_w_R(w0,t1_dot,R0_1.');
v1 = propagation_v_R(w0, v0, P0_1, R0_1.');
vc1 = simplify(propagation_v_R(w1, v1, P1_c1, R0_1));


% link 2 (R)
w2 = propagation_w_R(w1,t2_dot,R1_2.');
v2 = propagation_v_R(w1, v1, P1_2, R1_2.');
vc2 = simplify(propagation_v_R(w2, v2, P2_c2, R0_1*R1_2));


% link 3 (P)
w3 = propagation_w_P(w2,R2_3.');
v3 = propagation_v_P(w2, v2, d3_dot, P2_3, R2_3.');
vc3 = simplify(propagation_v_R(w3, v3, P3_c3, R0_1*R1_2*R2_3));


% link 4 (R)
w4 = propagation_w_R(w3,t4_dot,R3_4.');
v4 = propagation_v_R(w3, v3, P3_4, R3_4.')
vc4 = simplify(propagation_v_R(w4, v4, P4_c4, R0_1*R1_2*R2_3*R3_4));

%% Kinetic Energy

T1 = simplify(kinetic_energy(w1, vc1, m1, I1));
T2 = simplify(kinetic_energy(w2, vc2, m2, I2));
T3 = simplify(kinetic_energy(w3, vc3, m3, I3));
T4 = simplify(kinetic_energy(w4, vc4, m4, I4));
T = T1 + T2 + T3 + T4;
%% Potiential  Energy

G = [0 0 g].';
Pc1 = (T0_1 * [P1_c1;1]);
Pc1 = Pc1(1:3);
Pc2 = (T0_1*T1_2 * [P2_c2;1]);
Pc2 = Pc2(1:3);
Pc3 = (T0_1*T1_2*T2_3 * [P3_c3;1]);
Pc3 = Pc3(1:3);
Pc4 = (T0_1*T1_2*T2_3*T3_4*[P1_c1;1]);
Pc4 = Pc4(1:3);

U1 = simplify(potiential_energy(Pc1, G, m1))
U2 = simplify(potiential_energy(Pc2, G, m2))
U3 = simplify(potiential_energy(Pc3, G, m3))
U4 = simplify(potiential_energy(Pc4, G, m4))
U = U1 + U2 + U3 + U4;



%% Differential operations

% define Array for substituting symbolic functions to syms
subsArraySymfunc = [diff(t1_dot,t),diff(t2_dot,t),diff(d3_dot,t),diff(t4_dot,t),t1_dot, t2_dot, d3_dot, t4_dot, t1, t2, d3, t4];
subsArraySymvar = [t1_ddot_, t2_ddot_, d3_ddot_, t4_ddot_, t1_dot_, t2_dot_, d3_dot_, t4_dot_, t1_, t2_, d3_, t4_];

T_ = subs(T, subsArraySymfunc, subsArraySymvar);
U_ = subs(U, subsArraySymfunc, subsArraySymvar);


DT_Dt1 =  diff(T_, t1_);
DT_Dt2 =  diff(T_, t2_);
DT_Dd3 =  diff(T_, d3_);
DT_Dt4 =  diff(T_, t4_);
          
DT_Dt1_dot =  diff(T_, t1_dot_);
DT_Dt2_dot =  diff(T_, t2_dot_);
DT_Dd3_dot =  diff(T_, d3_dot_);
DT_Dt4_dot =  diff(T_, t4_dot_);
                          
Dt1_dot_Dt =      diff(subs(DT_Dt1_dot, subsArraySymvar, subsArraySymfunc), t);
Dt2_dot_Dt =      diff(subs(DT_Dt2_dot, subsArraySymvar, subsArraySymfunc), t);
Dd3_dot_Dt =      diff(subs(DT_Dd3_dot, subsArraySymvar, subsArraySymfunc), t);
Dt4_dot_Dt =      diff(subs(DT_Dt4_dot, subsArraySymvar, subsArraySymfunc), t);

DU_Dt = [diff(U_, t1_);
         diff(U_, t2_);
         diff(U_, d3_);
         diff(U_, t4_)];
     
%% Equations of Motion

tau1 = Dt1_dot_Dt - DT_Dt1 + DU_Dt(1);
tau2 = Dt2_dot_Dt - DT_Dt2 + DU_Dt(2);
tau3 = Dd3_dot_Dt - DT_Dd3 + DU_Dt(3);
tau4 = Dt4_dot_Dt - DT_Dt4 + DU_Dt(4);
% for display
tau1_ = subs(tau1, subsArraySymfunc, subsArraySymvar)
tau2_ = subs(tau2, subsArraySymfunc, subsArraySymvar)
tau3_ = subs(tau3, subsArraySymfunc, subsArraySymvar)
tau4_ = subs(tau4, subsArraySymfunc, subsArraySymvar)


