function  [Positions] = ForwardKinematics(JointAngles, DH_table, flat_drawAxis)
    dof = size(DH_table,1);
    T = eye(4);
    Positions = zeros(3, dof + 1);
    Positions(:,1) = [0;0;0];
  
    for i = 1 : dof
        jointAngle = JointAngles(i);
        DH_params = DH_table(i, :);
        A = GenerateTransformationMatrices(jointAngle, DH_params);
        T = T * A;
        RotationMatrix = T(1:3, 1:3);
        PositionVector = T(1:3, 4);
        if flat_drawAxis
        drawAxis(PositionVector, RotationMatrix, 2);
        end
        % return
        Positions(:, i+1) = PositionVector
       
    end
    
end
