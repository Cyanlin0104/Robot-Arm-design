function v_dot_ = propagation_v_dot_R(w, w_dot, v_dot, p, rotation_matrix)
v_dot_ = rotation_matrix * (v_dot + cross(w_dot, p) + cross(w, cross(w, p)));
end