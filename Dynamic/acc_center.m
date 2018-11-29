function acceleration = acc_center(previous_acc,previous_ang_vel,previous_angular_acc,Pc)
acceleration=previous_acc+cross(previous_angular_acc,Pc)+cross(previous_ang_vel,cross(previous_ang_vel,Pc));
end