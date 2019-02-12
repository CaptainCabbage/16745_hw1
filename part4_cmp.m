function part4_cmp( q0, target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles)
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

%optimization here 

cost = @(q)criterion2(q,link_length,twists, gst0, target, obstacles, lb, ub);
con = @(q)constraints(q, link_length, target,twists, gst0, obstacles);


%sqp 

options = optimoptions('fmincon', 'ConstraintTolerance',1e-3,'MaxFunEvals',10000000,...
    'SpecifyObjectiveGradient', true, 'Algorithm','sqp');


tic
[p_sqp, fval_sqp, flag_sqp, output_sqp]=fmincon(cost,q0,[],[],[],[],lb,ub,con,options);
time_sqp = toc;

err = norm(ForwardKinematics(p_sqp, twists,gst0) - target);
fprintf('sqp:');
fprintf('time: %f', time_sqp);
if err < 0.001
    fprintf('the goal is reached, error %f.', err);
else
    fprintf('cannot reach the goal, distance %f.', err);
end

fprintf('fval %f, num iter %d \n', fval_sqp, output_sqp.iterations);

% 'interior-point'

options = optimoptions('fmincon', 'ConstraintTolerance',1e-3,'MaxFunEvals',10000000,...
    'SpecifyObjectiveGradient', true, 'Algorithm','interior-point');


tic
[p_ip, fval_ip, flag_ip, output_ip]=fmincon(cost,q0,[],[],[],[],lb,ub,con,options);
time_ip = toc;
fprintf('interior-point:');
err = norm(ForwardKinematics(p_ip, twists,gst0) - target);
fprintf('time: %f', time_ip);

if err < 0.001
    fprintf('the goal is reached, error %f.', err);
else
    fprintf('cannot reach the goal, distance %f.', err);
end

fprintf('fval %f, num iter %d \n', fval_ip, output_ip.iterations);

% 'active-set'
options = optimoptions('fmincon', 'ConstraintTolerance',1e-3,'MaxFunEvals',10000000,...% 'MaxIterations', 600, ...
    'SpecifyObjectiveGradient', true, 'Algorithm','active-set');
tic
[p_as, fval_as, flag_as, output_as]=fmincon(cost,q0,[],[],[],[],lb,ub,con,options);
time_as = toc;
fprintf('active-set:');
err = norm(ForwardKinematics(p_as, twists,gst0) - target);
fprintf('time: %f', time_as);

if err < 0.001
    fprintf('the goal is reached, error %f.', err);
else
    fprintf('cannot reach the goal, distance %f.', err);
end

fprintf('fval %f, num iter %d \n', fval_as, output_as.iterations);

% cmaes

cost_cmaes = @(x)crit_cmaes(x,link_length,twists, gst0, target,obstacles, lb, ub);
opts.LogPlot = 'off';
opts.LBounds = lb; 
opts.UBounds = ub; 
%opts.MaxFunEvals = 500;
%opts.MaxIter = 20;
tic
[p_cmaes, fval_cmaes, counteval_cmaes, stopflag_cmaes, out_cmaes, bestever_cmaes] = cmaes(cost_cmaes,q0 , pi, opts);
time_cmaes = toc;
err = norm(ForwardKinematics(p_cmaes, twists,gst0) - target);
fprintf('cmaes:');
fprintf('time: %f', time_cmaes);

if err < 0.001
    fprintf('the goal is reached, error %f.', err);
else
    fprintf('cannot reach the goal, distance %f.', err);
end

fprintf('fval %f \n', fval_cmaes);


end