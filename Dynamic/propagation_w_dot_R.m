function w_dot_ = propagation_w_dot_R(w, w_dot, theta_dot, theta_ddot, rotation_matrix)
w_dot_ = rotation_matrix * w_dot + ...
    cross(rotation_matrix * w ,[0;0;theta_dot]) + [0;0;theta_ddot];
end