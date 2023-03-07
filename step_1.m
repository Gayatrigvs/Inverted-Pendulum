t = 0:0.05:10;
u = ones(size(t));
[y,t] = lsim(sys_tf,u,t);
plot(t,y)
title('Open-Loop Step Response')
axis([0 3 0 50])
legend('x','phi')

step_info = lsiminfo(y,t);
cart_info = step_info(1)
pend_info = step_info(2)