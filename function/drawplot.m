function drawplot(tlist,save_q1,save_q2,save_q1dot,save_q2dot,desired_q1,desired_q1dot,desired_q2,desired_q2dot)
% plot
figure(1);

subplot(4,2,1);
plot(tlist,save_q1,'b-');
hold on;
plot(tlist,desired_q1,'r:')
axis([0,max(tlist),min(desired_q1),max(desired_q1)]);
title("\theta_{1}");
hold off;
subplot(4,2,3);
plot(tlist,save_q1dot,'b-');
hold on;
plot(tlist,desired_q1dot,'r:');
axis([0,max(tlist),min(desired_q1dot),max(desired_q1dot)]);

t1 = title("$$\dot{\theta_{1}}$$");
set(t1,'Interpreter','latex');
hold off;
subplot(4,2,5);
plot(tlist,save_q2,'b-');
hold on;
plot(tlist,desired_q2,'r:');
axis([0,max(tlist),min(desired_q2),max(desired_q2)])

title("\theta_{2}");
hold off;
subplot(4,2,7);
plot(tlist,save_q2dot,'b-');
hold on;
plot(tlist,desired_q2dot,'r:');
axis([0,max(tlist),min(desired_q2dot),max(desired_q2dot)])
t2=title("$$\dot{\theta_{2}}$$");
set(t2,'Interpreter','latex');
hold off;
end