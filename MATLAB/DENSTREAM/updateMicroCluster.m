function [ mc ] = updateMicroCluster( P,mc,lambda,t_now,mc_type )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% define fading function
fading = @(t) 2^(-lambda*(t_now - t));

PRINT_RADIUS = 0;

%% BEGIN FUNCTION

% add point to micro-cluster
if isempty(mc)
    % add point to NEW micro-cluster
    mc.points = P;
    
    % add 't0' field to NEW o-micro-cluster
    if strcmp(mc_type,'o')
        mc.t0 = t_now;
    end
else
    % add point to existing micro-cluster
    mc.points = [mc.points; P];
end

% init micro-cluster incremental parameters
mc.weight = 0;
mc.CF1    = zeros(1,2);
mc.CF2    = zeros(1,2);

% calculate p-micro-cluster parameters, {w,CF_1,CF_2}
for j=1:size(mc.points,1)
    mc.weight = mc.weight + fading(mc.points(j,4));
    mc.CF1    = mc.CF1 + fading(mc.points(j,4))*mc.points(j,1:2);
    mc.CF2    = mc.CF2 + fading(mc.points(j,4))*mc.points(j,1:2).^2;
end

% calculate p-micro-cluster parameters, {center,radius} = f(w,CF1,CF2)
mc.center = mc.CF1 / mc.weight;
mc.radius = sqrt(norm(mc.CF2) / mc.weight - ...
                (norm(mc.CF1) / mc.weight)^2);  % not working...
if PRINT_RADIUS
    fprintf('\nmc.radius =\n');
    disp(mc.radius)
end

% radius measurement with switched terms
mc.radius = sqrt((norm(mc.CF1) / mc.weight)^2 - ...
                  norm(mc.CF2) / mc.weight);
if PRINT_RADIUS
    disp(mc.radius)
end

% unweighted radius measurement
dist = pdist2(mc.center,mc.points(:,1:2),'euclidean');
mc.radius = max(dist);
if PRINT_RADIUS
    disp(mc.radius)
end

end

