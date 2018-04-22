v = 10.*[0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1 ;1 0 1; 1 1 1; 0 1 1];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
p1=[5,5,10]; p2=[5,5,20];p3=[5,15,50];p4=[5,20,60];p5=[5,45,40];p6=[5,45,30];

hold on;
patch('Vertices', v, 'Faces', f, 'FaceColor', 'green');axis equal;
plotLP(p1,p2);plotLP(p2,p3);plotLP(p3,p4);plotLP(p4,p5);plotLP(p5,p6);
view(3)

