function angular_velocity = ang_vel_revolute(previous_angular_velocity,rotation_matrix,theta_dot)
angular_velocity=previous_angular_velocity+rotation_matrix*[0;0;theta_dot];
end