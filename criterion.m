function [score] = criterion(p,link_lengths,twists, gst0, target,obstacles, lb, ub)
%target 3x1
% p joint angles

draw(target, link_lengths, p,twists, obstacles);
[pos] = ForwardKinematics(p,twists,gst0);

score = 1000*norm(target - pos)^2 - (norm(p - lb)^2 + norm(ub - p)^2);
%score =  -norm(p - lb)^2 - norm(ub - p)^2;
end

