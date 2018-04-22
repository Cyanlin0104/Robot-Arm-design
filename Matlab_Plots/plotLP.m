function [] = plotLP(p1, p2)
    line('xData',[p1(1),p2(1)],'yData',[p1(2),p2(2)],...
        'zData',[p1(3),p2(3)],'Color','r','lineWidth',3);
    plot3(p1(1),p1(2),p1(3),'ro');
    plot3(p2(1),p2(2),p2(3),'ro');
