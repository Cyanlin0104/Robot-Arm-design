function acceleration = acc_pris(previous_acc,rotation_matrix,w,w_dot,d_dot,d_ddot,P,w_current)
acceleration=previous_acc+cross(w_dot,P)+cross(w,cross(w,P))+2*cross(w_current,rotation_matrix'*[0;0;d_dot])+rotation_matrix'*[0;0;d_ddot]
end