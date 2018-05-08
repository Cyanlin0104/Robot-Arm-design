function [] = drawAxis(Origin, RotationMatrix, size)
    hold on;
    p2_x = Origin + size*RotationMatrix(:,1);
    p2_y = Origin + size*RotationMatrix(:,2);
    p2_z = Origin + size*RotationMatrix(:,3);
    line([Origin(1), p2_x(1)],[Origin(2),p2_x(2)],...
         [Origin(3), p2_x(3)], 'Color','r');hold on;
    line([Origin(1), p2_y(1)],[Origin(2),p2_y(2)],...
         [Origin(3), p2_y(3)], 'Color','g');hold on;
    line([Origin(1), p2_z(1)],[Origin(2),p2_z(2)],...
         [Origin(3), p2_z(3)], 'Color','b');hold on;
    view(3);
    grid on;
end