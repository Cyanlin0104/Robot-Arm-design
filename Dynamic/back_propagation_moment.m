function moment = back_propagation_moment(moment_, f_, N, F, P, Pc,rotation_matrix)
moment =rotation_matrix*moment_ + N + cross(Pc,F)+cross(P,rotation_matrix*f_);
end