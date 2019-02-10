function [score, g] = criterion2(p,link_lengths,twists, gst0, target,obstacles, lb, ub)
%target 3x1
% p joint angles
N = numel(p)/3;
P = reshape(p,3,N)';
rolls = P(:,1);
pitches = P(:,2);
yaws = P(:,3);
draw(target, link_lengths, p,twists, obstacles);
[pos] = ForwardKinematics(p,twists,gst0);

J_twist = compute_Jacobians(p,twists);
J = eye(7,6);
q = pos(4:end);
J(4:7, 4:6) = 0.5*[-q(2), -q(3), -q(4); q(1),-q(4),q(3);q(4),q(1),-q(2);-q(3),q(2),q(1)];
J = J*J_twist;
gd = (abs(target - pos)'*J)';
score = norm(target - pos)^2;% - norm(p - lb)^2 - norm(ub - p)^2;
g = gd;
%score = 100*norm(target - pos)^2 -norm(p - lb)^2 - norm(ub - p)^2;
%g = 100*gd - abs(p - lb) - abs(ub - p);
end

