clc;
clear all;
close all;

%Define modal parameters for "measured" FRF
fn1 = 500;   % Hz
wn1 = fn1 *2*pi; % rad/s
zetaq1 = 0.09;
kq1 = 8e6;  % N/m

fn2 = 760;   %Hz
wn2 = fn2 *2*pi; % rad/s
zetaq2 = 0.05;
kq2 = 4e6;  % N/m

fn3 = 850;   %Hz
wn3 = fn3 *2*pi; % rad/s
zetaq3 = 0.03;
kq3 = 5e6;  % N/m

%Define measured FRF
w = (0:0.2:1500)'*2*pi; %frequency,rad/s
r1 = w/wn1;
r2 = w/wn2;
r3 = w/wn3;

Q1_R1 = (1/kq1)*((1-r1.^2) - 1i*(2*zetaq1*r1)) ./ ((1-r1.^2).^2 + (2*zetaq1*r1).^2); % 1i  =  i
Q2_R2 = (1/kq2)*((1-r2.^2) - 1i*(2*zetaq2*r2)) ./ ((1-r2.^2).^2 + (2*zetaq2*r2).^2); 
Q3_R3 = (1/kq3)*((1-r3.^2) - 1i*(2*zetaq3*r3)) ./ ((1-r3.^2).^2 + (2*zetaq3*r3).^2); 

FRF = Q1_R1 + Q2_R2 + Q3_R3;



%Add noise to simulate actual measurement
noise = 0.025*randn(length(FRF),1).*abs(FRF);
FRF = FRF + noise + 1i*noise;


figure(1)
subplot(211)
plot(w/2/pi, real(FRF), 'r')
ylim([-3e-6 2.5e-6])
set(gca,'FontSize', 14)
ylabel('Real (m/N)')
hold on
subplot(212)
plot(w/2/pi, imag(FRF), 'r')
ylim([-4.5e-6 4.5e-7])
set(gca,'FontSize', 14)
xlabel('Frequency (Hz)')
ylabel('Imag (m/N)')
hold on


% Perfrom fit
fn1 = 499;    % from Im plot 
wn1 = fn1 *2*pi;  
zetaq1 = (533 - 460)*2*pi/(2*wn1);  %from Re plot,  1 + zetaq1 = 533;  1 - zetaq1  = 460
kq1 = 1/(2*zetaq1*7.62e-7);         %from Im plot, when r1=1, Im = 7.62e-7  

fn2 = 761;
wn2 = fn2 *2*pi;
zetaq2 = (787 - 726)*2*pi/(2*wn2);
kq2 = 1/(2*zetaq2*2.77e-6);

fn3 = 849;
wn3 = fn3 *2*pi;
zetaq3 = (877 - 828)*2*pi/(2*wn3);
kq3 = 1/(2*zetaq3*3.74e-6);

r1 = w/wn1;
r2 = w/wn2;
r3 = w/wn3;

FRF1 = (1/kq1)*((1-r1.^2) - 1i*(2*zetaq1*r1)) ./ ((1-r1.^2).^2 + (2*zetaq1*r1).^2);
FRF2 = (1/kq2)*((1-r2.^2) - 1i*(2*zetaq2*r2)) ./ ((1-r2.^2).^2 + (2*zetaq2*r2).^2); 
FRF3 = (1/kq3)*((1-r3.^2) - 1i*(2*zetaq3*r3)) ./ ((1-r3.^2).^2 + (2*zetaq3*r3).^2); 

FRF = FRF1 + FRF2 + FRF3; % model fit

figure(2)
subplot(211)
plot(w/2/pi, real(FRF1), 'b:', w/2/pi, real(FRF2), 'b:', w/2/pi, real(FRF3), 'b:')
subplot(212)
plot(w/2/pi, imag(FRF1), 'b:', w/2/pi, imag(FRF2), 'b:', w/2/pi, imag(FRF3), 'b:')

figure(3)
subplot(211)
plot(w/2/pi, real(FRF), 'b:')
subplot(212)
plot(w/2/pi, imag(FRF), 'b:')


