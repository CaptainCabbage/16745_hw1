function [g] = transformation(link_l, roll, pitch, yaw)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
r = eul2rotm([roll, pitch, yaw], 'XYZ');
g = [r, r*[link_l, 0,0]'; 0 0 0 1];
end

