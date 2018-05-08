function  JointAngles = InverseKinematicsForXYZ(Link_length, Target_point)
%{
    Args:
        Inputs:
            Link_length: 
                        An array in which element should be a interger 
                            that represent length of each link.
                        (Note that the size of array should be dof).
            Target_points:
                         An array in which each element indicates the spacial 
                            targets position (x, y, z) of the end effector.
        
        Output:
            JointAngles:    
                        An array describe that angles each joint should
                        rotate to reach the target poisiton.
    

%}
xc = Target_point(1)
yc = Target_point(2)
zc = Target_point(3)
L1 = Link_length(1)
L2 = Link_length(2)
L3 = Link_length(3)


theta1 = atan2(yc,xc);
C_thetaD = (L2^2 + L3^2 - (xc^2 + yc^2) - (zc - L1)^2) / (2*L2*L3);
theta3 =  acos(C_thetaD) - pi;
theta2 = atan2(zc-L1,sqrt(xc^2 + yc^2)) - atan2(L3*sin(theta3),L2+L3*cos(theta3));
% return 
JointAngles = [theta1 theta2 theta3]
end   

