function drawrobot(q)


global l1;
global l2;
c1 = cos(q(1));
c12 = cos(q(1)+q(2));
s1 = sin(q(1));
s12 = sin(q(1)+q(2));

p1 = [l1*c1,l1*s1];
p2 = [l1*c1+l2*c12,l1*s1+l2*s12];

plot([0 p1(1) p2(1)],[0 p1(2) p2(2)],'k-');
hold on;
plot([0 p1(1) p2(1)],[0 p1(2) p2(2)],'k.',"MarkerSize",10);
axis([-2.5,2.5,-2.5,2.5]);
grid on;
daspect([1,1,1]);
drawnow limitrate;

hold off;
end