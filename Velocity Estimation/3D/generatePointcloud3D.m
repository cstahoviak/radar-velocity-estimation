function [ pointcloud ] = generatePointcloud3D( Ntargets )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

min_x = 0;
max_x = 10;
ptcloud_x = (max_x - min_x).*rand(Ntargets,1) + min_x;

min_y = -10;
max_y = 10;
ptcloud_y = (max_y - min_y).*rand(Ntargets,1) + min_y;

min_z = -10;
max_z = 10;
ptcloud_z = (max_z - min_z).*rand(Ntargets,1) + min_z;

pointcloud = [ptcloud_x, ptcloud_y, ptcloud_z];

end

