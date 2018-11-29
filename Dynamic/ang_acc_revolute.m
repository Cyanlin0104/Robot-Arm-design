function angular_acceleration = ang_acc_revolute(previous_angular_acc,previous_angular_velocity,rotation_matrix,theta_dot,theta_ddot)
angular_acceleration=rotation_matrix*previous_angular_acc+...
    cross(rotation_matrix*previous_angular_velocity,[0;0;theta_dot])+[0;0;theta_ddot];
end