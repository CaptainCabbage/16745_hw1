function [score, g] = criterion2(p,link_lengths,twists, gst0, target,obstacles, lb, ub)
%target 3x1
% p joint angles

draw(target, link_lengths, p,twists, obstacles);
[pos] = ForwardKinematics(p,twists,gst0);

J_twist = compute_Jacobians(p,twists);
J = eye(7,6);
q = pos(4:end);
%J(4:7, 4:6) = 0.5*[-q(2), -q(3), -q(4); q(1),-q(4),q(3);q(4),q(1),-q(2);-q(3),q(2),q(1)];
J(4:7, 4:6) = 0.5*[-q(2), -q(3), -q(4); q(1),q(4),-q(3);-q(4),q(1),q(2);q(3),-q(2),q(1)];
J = J*J_twist;
gd = ((pos - target)'*J)';
%score = norm(target - pos)^2;% - norm(p - lb)^2 - norm(ub - p)^2;
%g = gd;
score = 1000*norm(target - pos)^2 -norm(p - lb)^2 - norm(ub - p)^2;
g = 1000*gd - ((p - lb) +(p - ub));
end

