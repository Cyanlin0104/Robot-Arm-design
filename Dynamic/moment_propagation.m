function torque= moment_propagation(rotation_matrix, n_next,N_current,F_current,P,Pc,f_next)
torque=rotation_matrix'*n_next+N_current+cross(Pc,F_current)+cross(P,rotation_matrix'*f_next);
end