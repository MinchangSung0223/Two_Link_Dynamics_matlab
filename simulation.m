%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  PBL-based Robot Control
%  cyj@hanyang.ac.kr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all
home

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
save_q = []
save_qdot = []
save_u = []
% 그림을 몇번에 한번씩 보여줄 지
skip_count = 10;
for i = 0 : s_time : tf
   u=0;

   % Forward Dynamics 부분
   [t,y] = ode45(@(t,q) two_link_dynamics(t,q),[0, s_time] , [q(1) q(2) qdot(1) qdot(2)] );
   index = size(y);
   q(1) = y(index(1), 1);
   q(2) = y(index(1), 2);
   qdot(1) = y(index(1), 3);
   qdot(2) = y(index(1), 4);

   % 시간마다 데이터 저장
   save_q = [save_q,q'];
   save_qdot = [save_qdot,qdot'];
   save_u = [save_u,u];
   
   % 그림그리기
   if(mod(n,skip_count)==0)
         drawrobot(q);
   end

   n=n+1;
   
end