function [r, p, y] = part1( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
%% Function that uses optimization to do inverse kinematics for a snake robot

%%Outputs 
  % [r, p, y] = roll, pitch, yaw vectors of the N joint angles
  %            (N link coordinate frames)
%%Inputs:
    % target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
    %    effector
    % link_length : Nx1 vectors of the lengths of the links
    % min_xxx, max_xxx are the vectors of the 
    %    limits on the roll, pitch, yaw of each link.
    % limits for a joint could be something like [-pi, pi]
    % obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
    %    obstacle. M obstacles.

% Your code goes here.
target(4:end) = target(4:end)/norm(target(4:end));
N = numel(link_length);
q0 = unifrnd(-pi,pi,3*N,1);

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

%options = optimoptions('fmincon','SpecifyObjectiveGradient',true)
options = optimset('Display','iter','MaxFunEvals',100000,'Algorithm','sqp');
cost = @(q)criterion(q,link_length,twists, gst0, target, obstacles, lb, ub);
con = @(q)constraints(q, link_length, target,twists, gst0, obstacles);
[p_final, fval, ~, output]=fmincon(cost,q0,[],[],[],[],lb,ub,con,options);

err = norm(ForwardKinematics(p_final, twists,gst0) - target);

if err < 0.1
    fprintf('the goal is reached, error %f', err);
else
    fprintf('cannot reach the goal, distance %f', err);
end

fprintf('fval: %f \n', fval);
fprintf('iters: %d \n', output.iterations)

P = reshape(p_final,3,N)';
r = P(:,1);
p = P(:,2);
y = P(:,3);

end
