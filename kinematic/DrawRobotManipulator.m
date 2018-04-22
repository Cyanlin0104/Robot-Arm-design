function  DrawRobotManipulator(Positions)
size_ = size(Positions);
num_points = size_(1);
for i = 1 : num_points - 1
    plotLP(Positions(i,:),Positions(i+1,:))
end

end