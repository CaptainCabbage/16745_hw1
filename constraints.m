function [c,ceq] = constraints(p,link_lengths, target,twists, gst0, obstacles)

c = -minDistObs(p,link_lengths, obstacles);

[pos] = ForwardKinematics(p,twists,gst0);

ceq = pos - target;
%ceq = [];
end

