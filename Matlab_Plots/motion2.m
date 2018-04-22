x = 0:0.1:8*pi;
for i = 1:5000
    h = sin(x-i/50).*exp(-x/5);
    plot(x,h);
    axis([-inf inf -1 1]);
    grid on;
    drawnow
end