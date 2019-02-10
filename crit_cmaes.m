function [score] = crit_cmaes(p,link_lengths,twists, gst0, target,obstacles, lb, ub)

c = minDistObs(p,link_lengths, obstacles);
if sum(c<=0) > 0 %|| sum(p < lb) > 0 || sum(p>ub) >0
    score = NaN;
    return
end

draw(target, link_lengths, p,twists, obstacles);
[pos] = ForwardKinematics(p,twists,gst0);

score = norm(target - pos)^2;% - norm(p - lb)^2 - norm(ub - p)^2;
%score =  -norm(p - lb)^2 - norm(ub - p)^2;

end

