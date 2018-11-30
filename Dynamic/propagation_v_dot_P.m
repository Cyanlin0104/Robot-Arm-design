function v_dot_ = propagation_v_dot_P(w, w_dot, v_dot, d_dot, d_ddot, p, rotation_matrix)
v_dot_ = rotation_matrix * (v_dot + cross(w_dot, p) + cross(w, cross(w, p)) + ...
    2*cross(w, rotation_matrix.'*[0;0;d_dot]) + rotation_matrix.'*[0;0;d_ddot]);
end