function [minD] = minDistObs(p,link_lengths, obstacles)
%minD Mx1
N = numel(link_lengths);
M = size(obstacles,1);
if M == 0
    minD = [];
    return
end
g = eye(4);
v1 = [0,0,0]';
D = zeros(N,M);
ob_p = obstacles(:,1:3);
ob_r = obstacles(:,4);

rolls = p(1:N);
pitches = p(N+1:2*N);
yaws = p(2*N+1:end);

for i = 1:N
    g = g*transformation(link_lengths(i), rolls(i), pitches(i), yaws(i));
    v2 = g(1:3,4);
    d = point_to_line(ob_p, v1', v2')';
    D(i,:) = d;
    v1 = v2;
end
minD = min(D)' - ob_r;

end

