function [traj,dtraj,ddtraj] = JointTrajectory(thetastart, thetaend, Tf, s_time)
N = Tf/s_time
timegap = Tf / (N - 1);
traj = zeros(size(thetastart, 1), N);
dtraj = zeros(size(0, 1), N);
ddtraj = zeros(size(0, 1), N);

for i = 1: N
     s = CubicTimeScaling(Tf, timegap * (i - 1));
     traj(:, i) = thetastart + s * (thetaend - thetastart);
     if i>2
        dtraj(:, i) = (traj(:, i)-traj(:, i-1))/s_time;
        ddtraj(:, i) =(dtraj(:, i)-dtraj(:, i-1))/s_time;
     end
     
end
traj = traj';
end
function s = CubicTimeScaling(Tf, t)
s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3;

end

