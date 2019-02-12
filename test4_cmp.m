
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

N=5;
%%
q2 = unifrnd(-pi,pi,3*N,1);
part4_cmp(q2, target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles)