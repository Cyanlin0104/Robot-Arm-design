x=0:0.1:3*pi; z1=sin(x); z2=sin(2.*x);z3=sin(3.*x);
y1=zeros(size(x)); y3=ones(size(x)); y2=y3./2;
plot3(x,y1,z1,'r',x,y2,z2,'b',x,y3,z3,'g');grid on;
