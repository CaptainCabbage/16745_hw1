function [r, p, y] = part4( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
%% multiple optima

target(4:end) = target(4:end)/norm(target(4:end));
N = numel(link_length);

twists = zeros(6,3*N);
p = [0;0;0];
for i = 1:N
    if i ~= 1
        p(1) = p(1) + link_length(i-1);
    end
    for j = 1:3
        w = zeros(3,1);
        w(j) = 1;
        twist = compute_twist(w,p);
        twists(:,3*(i-1)+j) = twist;
    end
end
gst0 = [eye(3),[sum(link_length);0;0];0 0 0 1];

lb = [min_roll, min_pitch, min_yaw]';
lb = lb(:);
ub = [max_roll, max_pitch, max_yaw]';
ub = ub(:);

% optimization here
n_trial = 5;
fvals = zeros(n_trial,1);

parfor k = 1:n_trial
    q0{k} = unifrnd(-pi,pi,3*N,1);

    options = optimoptions('fmincon','Display','iter','MaxFunEvals',1000000,...
        'SpecifyObjectiveGradient', true, 'Algorithm','sqp');
    cost = @(q)criterion2(q,link_length,twists, gst0, target, obstacles, lb, ub);
    con = @(q)constraints2(q, link_length, target,twists, gst0, obstacles);
    [p_finals{k}, fvals(k)]=fmincon(cost,q0{k},[],[],[],[],lb,ub,con,options);

    %{
    err = norm(ForwardKinematics(p_finals(k), twists,gst0) - target);

    if err < 0.1
        fprintf('the goal is reached, error %f', err);
    else
        fprintf('cannot reach the goal, distance %f', err);
    end

    fprintf('fval %f \n', fvals(k));
    %}
end

[min_f,ind] = min(fvals);

fprintf('best %d th, fval: %f', ind, min_f);


P = reshape(p_final{ind},3,N)';
r = P(:,1);
p = P(:,2);
y = P(:,3);

end