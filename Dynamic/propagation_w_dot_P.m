function w_dot_ = propagation_w_dot_P(w_dot, rotation_matrix)
w_dot_ = rotation_matrix * w_dot;
end