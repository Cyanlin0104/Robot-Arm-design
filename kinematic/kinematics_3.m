function kinematics_3(Link_length, target_point, DH_table)
        JointAngles = InverseKinematicsForXYZ(Link_length, target_point)
        pos = ForwardKinematics(JointAngles, DH_table);
        DrawRobotManipulator(Link_length,pos);
end

 

