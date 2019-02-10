function [pos] = ForwardKinematics(p, twists,gst0)

% q: joint angle
% twists 6 x n 
% gwt0 4 x 4
% start point [0 0 0 1];


% J jacobian

num_joints = size(twists,2);
g = eye(4);
for j = 1:num_joints
    twist = twists(:,j);
    theta = p(j);
    g = g*exp_twist(twist,theta);
end
g = g*gst0;
r = g(1:3,1:3);
q = rotm2quat(r);
pos = g*[0 0 0 1]';
pos = [pos(1:3);q'];

end

