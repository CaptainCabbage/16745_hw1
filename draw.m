function draw(target, link_lengths, q, twists, obstacles)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
N = numel(link_lengths);
M = size(obstacles,1);

pos1 = [0,0,0]';
scatter3(0, 0, 0, 'g', 'filled');
hold on;
scatter3(target(1), target(2), target(3), 'b', 'filled');
hold on;
scatter3(pos1(1), pos1(2), pos1(3), 'b');
hold on;
axis([-5 5 -5 5 -5 5])
[x,y,z] = sphere;
hold on; 
for j = 1:M
  r = obstacles(j,4);
  surf(r*x+obstacles(j,1),r*y+obstacles(j,2),r*z+obstacles(j,3));
  hold on;
end
    
for i = 1:N
    g0 = eye(4);
    g0(1,4) = sum(link_lengths(1:i));
    [pos] = ForwardKinematics(q(1:3*i), twists(:,1:3*i),g0);
    pos2 = pos(1:3);
    plot3([pos1(1),pos2(1)], [pos1(2),pos2(2)], [pos1(3),pos2(3)],'r', 'LineWidth', 2);
    hold on;
    scatter3(pos2(1), pos2(2), pos2(3), 'b');
    hold on;
    pos1 = pos2;
end
hold off;
drawnow
% delay
 for j = 1:1000000
   z = sin(sqrt(100.0*j));
 end
end

