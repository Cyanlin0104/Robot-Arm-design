x = linspace(0,3); y=x.^2.*sin(x); plot(x,y);
line([2,2],[0,2^2*sin(2)]);
str = '$$ \int_{0}^{2} x^2\sin(x) dx $$';
text(0.25,3.5,str,'Interpreter','x')
