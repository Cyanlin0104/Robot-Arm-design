syms theta1 theta1_dot theta1_ddot
% w1 = angVelPropagation_R([0 0 0]', theta1_dot, R1_0')
% v1 = velPropagation_R([0 0 0]', [0 0 0]', [0 0 0]', R1_0')
% w2 = angVelPropagation_R(w1, theta2_dot, R2_1')
% v2 = velPropagation_R(w1, v1, [l1 0 0]', R2_1')

% initial conditions

% Outward Propagation

% Inward Propagation



function R_angularVel_ = angVelPropagation_R(angularVel, theta_dot, rotationMat)
R_angularVel_ = rotationMat*angularVel + theta_dot*[0 0 1]';
end

function P_angularVel_ = angVelPropagation_P(angularVel, rotationMat)
P_angularVel_ = rotationMat*angularVel;
end


function R_linearVel_ = velPropagation_R(angularVel, linearVel, P_, rotationMat)
R_linearVel_ = rotationMat*(linearVel + cross(angularVel, P_));
end

function P_linearVel_ = velPropagation_P(angularVel, linearVel, P_, rotationMat, d_dot)
P_linearVel_ =  rotationMat*(linearVel + cross(angularVel, P_)) + d_dot*[0 0 1]';
end

function R_angularAcc_ = angAccPropagation_R(angularAcc, angularVel, theta_dot, theta_ddot, rotationMat)
R_angularAcc_ = rotationMat*angularAcc + rotationMat*cross(angularVel, theta_dot*[0 0 1]') + theta_ddot*[0 0 1]';
end

function P_angularAcc_ = angAccPropagation_P(angularAcc,rotationMat)
P_angularAcc_ = rotationMat*angularAcc;
end

function R_linearAcc = accPropagation_R(angularAcc, angularVel, linearAcc, P_, rotationMat)
R_linearAcc = rotationMat*(linearAcc + cross(angularAcc, P_) + cross(angularVel,cross(angularVel, P_)));
end

function P_linearAcc = accPropagation_P(angularAcc, angularVel, linearAcc, P_, rotationMat, d_dot, d_ddot)
P_linearAcc = rotationMat*(linearAcc + cross(angularAcc, P_) + cross(angularVel,cross(angularVel, P_)) + 2*cross(angularVel, rotationMat'*d_dot*[0 0 1]') + rotationMat'*d_ddot*[0 0 1]');
end

function Fi = inertia_force(angularAcc, angularVel, linearAcc, Pc, mass)
ac = linearAcc + cross(angularAcc, Pc) + cross(angularVel, cross(angularVel, Pc));
Fi = mass*ac;
end

function Ni = inertia_moment(angularAcc, angularVel, inertia_tensor)
Ni = inertia_tensor*angularAcc + cross(angularVel, inertia_tensor*angularVel);
end