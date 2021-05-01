%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  PBL-based Robot Control
%  cyj@hanyang.ac.kr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all
home

addpath("function")
global M;
global C;
global G;
global u;
global m1;
global m2;
global I1;
global I2;
global lc1;
global lc2;
global l1;
global l2;
global g;
global w;
m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
I1 = 1;
I2 = 1;
g = 9.806;
lc1 = 1
lc2 = 1




% Sampling Time
s_time = 0.001;
% 종료 시각
tf = 5;
% 초기 값 설정.
q(1) = 0;
q(2) = 0;
qdot(1) = 0;
qdot(2) = 0;
qtwodot(1) = 0;
qtwodot(2) = 0;
% 데이터 저장을 위한 변수 n %
n=1;
% 데이터 저장용 변수들
save_q1 = []
save_q2 = []

save_q1dot = []
save_q2dot = []

save_u = []
% 그림을 몇번에 한번씩 보여줄 지
skip_count = 10;

[desired_q1,desired_q1dot,desired_q1twodot]=JointTrajectory(0,pi/2,tf,s_time);
[desired_q2,desired_q2dot,desired_q2twodot]=JointTrajectory(0,pi/2,tf,s_time);

sum_e =[0;0];
Kp = 100;
Ki = 100;

time_list_size = size(0 : s_time : tf-s_time);
save_q1 = -100*ones(time_list_size);
save_q1dot = -100*ones(time_list_size);
save_q2 =-100*ones(time_list_size);
save_q2dot = -100*ones(time_list_size);
save_u = -100*ones(time_list_size);


f1=figure(1);
f1.Position=[75 331 1742  647];
pause(3);
for i = 0 : s_time : tf-s_time
    
    
    M_hat = [m1*l1*l1 + m2*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(q(2))+I1+I2, m2*l2*l2 + m2*l1*l2*cos(q(2))+I2 ;
        m2*l2*l2 + m2*l1*l2*cos(q(2))+I2, m2*l2*l2+I2];
    C_hat = [ -m2*l1*l2*qdot(2)*sin(q(2)) , -m2*l1*l2*(qdot(1)+qdot(2))*sin(q(2)) ;
        m2*l1*l2*qdot(1)*sin(q(2)) , 0 ];
    G_hat = [ (m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2)) ; m2*g*l2*cos(q(1)+q(2)) ];
   
   w = rand*200;
   % Computed Torque Control 부분
   qd = [desired_q1(n),desired_q2(n)]';
   qd_dot = [desired_q1dot(n),desired_q2dot(n)]';
   qd_twodot = [desired_q1twodot(n),desired_q2twodot(n)]';
   q = [q(1),q(2)]';
   e = qd-q;
   edot = qd_dot-qdot';
   u=M_hat*(qd_twodot+Kp*edot+Ki*e)+C_hat*(qd_dot+Kp*e+Ki*sum_e)+G_hat;
   sum_e = sum_e+e*s_time;
  
   
   
   
   % Forward Dynamics 부분
   [t,y] = ode45(@(t,q) two_link_dynamics(t,q),[0, s_time] , [q(1) q(2) qdot(1) qdot(2)] );
   index = size(y);
   q(1) = y(index(1), 1);
   q(2) = y(index(1), 2);
   qdot(1) = y(index(1), 3);
   qdot(2) = y(index(1), 4);

   
   
   % 시간마다 데이터 저장
   save_q1(n) = q(1);
   save_q2(n) = q(2);
   save_q1dot(n) = qdot(1);
   save_q2dot(n) = qdot(2);
   
   
   % 그림그리기
   if(mod(n,skip_count)==0)
         drawrobot(q);
         drawplot([0 : s_time : tf-s_time],save_q1,save_q2,save_q1dot,save_q2dot,desired_q1,desired_q1dot,desired_q2,desired_q2dot);
   end
   n=n+1;   
end

