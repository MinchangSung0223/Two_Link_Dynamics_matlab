%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  PBL-based Robot Control
%  cyj@hanyang.ac.kr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function dydt = two_link(t,y)

global M;
global C;
global G;
global u;
global m1;
global m2;
global l1;
global l2;
global g;
global I1;
global I2;
% Dynamic Parameters 
m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
I1 = 1;
I2 = 1;
g = 9.806;

M = [m1*l1*l1 + m2*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(y(2))+I1+I2, m2*l2*l2 + m2*l1*l2*cos(y(2))+I2 ;
        m2*l2*l2 + m2*l1*l2*cos(y(2))+I2, m2*l2*l2+I2];
C = [ -m2*l1*l2*y(4)*sin(y(2)) , -m2*l1*l2*(y(3)+y(4))*sin(y(2)) ;
        m2*l1*l2*y(3)*sin(y(2)) , 0 ];
G = [ (m1+m2)*g*l1*cos(y(1)) + m2*g*l2*cos(y(1)+y(2)) ; m2*g*l2*cos(y(1)+y(2)) ];
   
temp1 = [y(3) ; y(4)] ;
temp2 = M\(-C*temp1 - G + u);

dydt = [y(3); y(4) ; temp2(1) ; temp2(2) ];