function [R, Rp, Rr, Ry] = GenerateRotationMatrix(EulerAngles, rotationOrder)
p = EulerAngles(1);
r = EulerAngles(2);
y = EulerAngles(3);

% along X axis
Rp = [    1      0          0;
          0   cos(p)  -sin(p);
          0   sin(p)   cos(p);]
% along Y axis
Rr = [ cos(r)    0     sin(r);
          0      1          0;
      -sin(r)    0     cos(r);]
% along Z axis
Ry = [ cos(y) -sin(y)        0;
       sin(y)  cos(y)        0;
           0       0         1;]
       
if rotationOrder == "xyz"
    R = Ry*Rr*Rp;
elseif rotationOrder == "yzx"
    R = Rp*Ry*Rr;
elseif rotationOrder == "yxz"
    R = Ry*Rp*Rr;
end

end