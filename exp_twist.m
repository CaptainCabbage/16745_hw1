function [exp_t] = exp_twist(twist,theta)
% twist 6x1

v = twist(1:3);
omega = twist(4:6);

omega_hat = [0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
R = eye(3) + omega_hat*sin(theta) + omega_hat*omega_hat*(1-cos(theta));
t = (eye(3) - R)*(omega_hat*v) + omega*(omega')*v*theta;
exp_t = [R,t;0 0 0 1];

end

