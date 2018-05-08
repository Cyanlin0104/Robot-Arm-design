function  DrawRobotManipulator(Link_length, Positions)
%{
 Args:
    Inputs:
        link_length: An array in which element should be a interger 
                        that represent length of each link.
                     (Note that the size of array should be dof.)
        Positions:  A (3 x dof+1) Matrix, in which each column is the
                        position of their corresponding joint.
                     (Note that the first column is [0 0 0]' representing 
                      the ground.)
    Outputs:
        None.
%}
dof = size(Positions, 2) - 1;
if dof ~= length(Link_length)
    length(Link_length)
    disp('the columns of Positions and length of Link_length should be dof.');
    return;
end
hold on;
for i = 1 : length(Link_length)
    plotLP(Positions(:,i),Positions(:,i+1))
end
% Base table
v = (sum(Link_length)/length(Link_length)).* 0.5 .* ...
    [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 0 ;1 -1 0; 1 1 0; -1 1 0];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices', v, 'Faces', f, 'FaceColor', 'black');axis equal;

max_length = sum(Link_length);
axis([-max_length max_length -max_length max_length -max_length max_length])
grid on;
view(3);
end