function [] = drawXYZ(Origin, RotationMatrix)
    hold on;
    p2_x = Origin + RotationMatrix(:,1)';
    p2_y = Origin + RotationMatrix(:,2)';
    p2_z = Origin + RotationMatrix(:,3)';
    line([Origin(1), p2_x(1)],[Origin(2),p2_x(2)],...
         [Origin(3), p2_x(3)]);hold on;
    line([Origin(1), p2_y(1)],[Origin(2),p2_y(2)],...
         [Origin(3), p2_y(3)]);hold on;
    line([Origin(1), p2_z(1)],[Origin(2),p2_z(2)],...
         [Origin(3), p2_z(3)]);hold on;
    view(3);
    grid on;
end