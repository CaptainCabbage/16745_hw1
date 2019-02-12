% case 1
target = [2,1.5,2,1,1,1,0]'; 
link_length = ones(5,1);
min_roll =-ones(5,1)*pi;
max_roll = ones(5,1)*pi;
min_pitch = -ones(5,1)*pi; 
max_pitch = ones(5,1)*pi;
min_yaw = -ones(5,1)*pi;
max_yaw = ones(5,1)*pi;
obstacles = [1,1,1,0.5];



%{
% case 2
target = [2,1.5,1.5,3,7,8,5]'; 
link_length = ones(5,1);
min_roll =-ones(5,1)*pi;
max_roll = ones(5,1)*pi;
min_pitch = -ones(5,1)*pi; 
max_pitch = ones(5,1)*pi;
min_yaw = -ones(5,1)*pi;
max_yaw = ones(5,1)*pi;
obstacles = [1,0.8,1,0.5; 2,2,2,0.3; 3,2,1,0.6; 1, -0.5, 0, 0.5];
%}

[r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles );