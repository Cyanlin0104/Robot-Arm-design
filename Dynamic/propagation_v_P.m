function v_ = propagation_v_P(w, v, d_dot, p, rotation_matrix)
v_ = rotation_matrix * (v + cross(w, p) + [0;0;d_dot]);
end