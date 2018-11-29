clc;
clear all;
close all;

%define modal parameters
wn1 = 350.46;  % rad/s
zetaq1 = 0.035;
kq1 = 2.782e5; % N/m

wn2 = 988.76; % rad/s
zetaq2 = 0.099;
kq2 = 1.750e6; % N/m

M = [2 0;0 1];
C = [200 -120; -120 120];
K = [1e6 -6e5; -6e5 6e5];



%Define direct FRF
w = (0:0.2:1500)'; % frequency,rad/s
r1 = w/wn1;
r2 = w/wn2;
Q1_R1 = (1/kq1)*((1-r1.^2) - 1i*(2*zetaq1*r1)) ./ ((1-r1.^2).^2 + (2*zetaq1*r1).^2); % 1i  =  i
%{
 fill in 
%}
Q2_R2 = (1/kq2)*((1-r2.^2) - 1i*(2*zetaq2*r2)) ./ ((1-r2.^2).^2 + (2*zetaq2*r2).^2); 

FRF_direct = Q1_R1 + Q2_R2;

% Define cross FRF

%{
 fill in 
%}
    % find p11 and p21
    m1 = M(1,1);
    k1_plus_k2 = K(1,1);
    k2 = K(2,2);
    s_2 = roots([2 2.2e6 2.4e11]);
    s1_2 = s_2(1);
    s2_2 = s_2(2);
    p11 = k2/(m1*s1_2 + (k1_plus_k2)) 
    p21 = k2/(m1*s2_2 + (k1_plus_k2))
   
FRF_cross = p11*Q1_R1 - p21*Q2_R2;

figure(1)
subplot(211)
plot(w,real(FRF_direct));grid;
ylim([-2.7e-5 3.05e-5])
set(gca,'FontSize', 14)
ylabel('Real(m/N)')
subplot(212)
plot(w,imag(FRF_direct));grid;
ylim([-5.5e-5 5.5e-6])
set(gca,'FontSize', 14)
xlabel('Frequency(rad/s)')
ylabel('Imag(m/N)')

figure(2)
subplot(211)
plot(w,real(FRF_cross));grid;
ylim([-2.7e-5 3.05e-5])
set(gca,'FontSize', 14)
ylabel('Real(m/N)')
subplot(212)
plot(w,imag(FRF_cross));grid;
ylim([-5.5e-5 5.5e-6])
set(gca,'FontSize', 14)
xlabel('Frequency(rad/s)')
ylabel('Imag(m/N)')



