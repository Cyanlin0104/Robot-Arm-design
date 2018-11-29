function acceleration = acc(previous_acc,rotation_matrix,w,w_dot,P)
acceleration=rotation_matrix*(previous_acc+cross(w_dot,P)+cross(w,cross(w,P)));
end