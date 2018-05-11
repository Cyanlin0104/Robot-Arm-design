function kinematics_3(Link_length, target_point, DH_table)
        JointAngles = InversePosKinematics(Link_length, target_point)
        [pos,R] = ForwardKinematics(JointAngles, DH_table, true);
        pos
        DrawRobotManipulator(Link_length,pos);
end

 

