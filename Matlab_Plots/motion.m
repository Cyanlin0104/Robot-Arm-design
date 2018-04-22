x = 0:0.1:8*pi;
h = plot(x, sin(x).*exp(-x/5), 'EraseMode', 'None');
axis([-inf inf -1 1]);		% 設定圖軸的範圍
grid on				% 畫出格線
for i = 1:5000
    y = sin(x-i/50).*exp(-x/5);
    set(h, 'ydata', y);		% 設定新的 y 座標
    drawnow			% 立即作圖
end
