function v_ = propagation_v_R(w, v, p, rotation_matrix)
v_ = rotation_matrix * (v + cross(w, p));
end