x=-4*pi:0.1:4*pi;
y=sin(x);
h=cos(x);
w=1./(1+exp(-x));
g=(1/(2*pi*2)^0.5).*exp((-1.*(x-2*pi).^2)./(2*2^2));
plot(x,y,'rs-',x,h,'gp-',x,w,'b-',x,g,'-','lineWidth',1);
legend('sin(x)','cos(x)','Sigmoid','Guass func');
title('Function plots');
xlabel('x=-4\pi to 4\pi');



