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
n_trial = 10;
%%
algor_type = 'sqp';
[p_finals_sqp, fvals_sqp, outputs_sqp, flags_sqp] = part4( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles, n_trial, algor_type);
%[r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles );
%%
aver_fvals_sqp = mean(fvals_sqp);
aver_iter_sqp = 0;
for i = 1:n_trial
    aver_iter_sqp = aver_iter_sqp + outputs_sqp{i}.iterations/n_trial;
end
success_rate_sqp = sum(flags_sqp == 1)/n_trial;
%%
algor_type = 'interior-point';
[p_finals_ip, fvals_ip, outputs_ip, flags_ip] = part4( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles, n_trial, algor_type);
%[r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles );
aver_fvals_ip = mean(fvals_ip);
aver_iter_ip = 0;
for i = 1:n_trial
    aver_iter_ip = aver_iter_ip + outputs_ip{i}.iterations/n_trial;
end
success_rate_ip = sum(flags_ip == 1)/n_trial;


algor_type = 'active-set';
[p_finals_as, fvals_as, outputs_as, flags_as] = part4( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles, n_trial, algor_type);
%[r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles );
aver_fvals_as = mean(fvals_as);
aver_iter_as = 0;
for i = 1:n_trial
    aver_iter_as = aver_iter_as + outputs_as{i}.iterations/n_trial;
end
success_rate_as = sum(flags_as == 1)/n_trial;

