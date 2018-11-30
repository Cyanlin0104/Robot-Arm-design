function w_ = propagation_w_R( w,theta_dot,rotation_matrix)
w_ = w + rotation_matrix*[0;0;theta_dot];
end