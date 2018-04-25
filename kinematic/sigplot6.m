function [] = sigplot6()
    function y = SinSignalfunc(A, f, t, phase)
        y = A * sin(2*pi*f*t + phase);
    end
        
Fs = 100;
t = 0:1/Fs:2;
A = 2;
f = 10;

y1 = SinSignalfunc(A, f, t, 0.2*pi);
y2 = SinSignalfunc(A, f, t, 0.5*pi);
y3 = SinSignalfunc(A, f, t, pi);

plot(t, y1, 'r', t, y2, 'g', t, y3, 'b');grid on;
title('Sin function with different phase');
xlabel('t');ylabel('y');

legend('t-y1','t-y2','t-y3');

end
