function [c,ceq] = constraints2(p,link_lengths, target,twists, gst0, obstacles)


c = -minDistObs(p,link_lengths, obstacles);
cg = [];

N = numel(link_lengths);
rolls = p(1:N);
pitches = p(N+1:2*N);
yaws = p(2*N+1:end);
[pos] = ForwardKinematics(p,twists,gst0);
%[pos] = fk(link_lengths, rolls, pitches, yaws);
ceq = pos - target;

J_twist = compute_Jacobians(p,twists);
J = eye(7,6);
q = pos(4:end);
J(4:7, 4:6) = 0.5*[-q(2), -q(3), -q(4); q(1),-q(4),q(3);q(4),q(1),-q(2);-q(3),q(2),q(1)];
J = J*J_twist;

%ceqg = J;
%ceq = [];
end

