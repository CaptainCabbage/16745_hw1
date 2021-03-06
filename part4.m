function [p_finals, fvals, outputs, flags] = part4( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles, n_trial, algor )
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

% multiple initialization

fvals = zeros(n_trial,1);
flags = zeros(n_trial,1);

%optimization here 

options = optimoptions('fmincon',...%'Display','iter',...
    'MaxFunEvals',10000000,...
    'SpecifyObjectiveGradient', true, 'Algorithm',algor);
cost = @(q)criterion2(q,link_length,twists, gst0, target, obstacles, lb, ub);
con = @(q)constraints(q, link_length, target,twists, gst0, obstacles);

for k = 1:n_trial
    q0{k} = unifrnd(-pi,pi,3*N,1);
    tic
    [p_finals{k}, fvals(k), flags(k), outputs{k}]=fmincon(cost,q0{k},[],[],[],[],lb,ub,con,options);
    toc
    
    err = norm(ForwardKinematics(p_finals{k}, twists,gst0) - target);

    if err < 0.001
        fprintf('%d th trial: the goal is reached, error %f.',k, err);
    else
        fprintf('%d th trial: cannot reach the goal, distance %f.',k, err);
    end

    fprintf('fval %f, num iter %d \n', fvals(k), outputs{k}.iterations);
    
end

[min_f,ind] = min(fvals);

fprintf('best %d th, fval: %f', ind, min_f);

end