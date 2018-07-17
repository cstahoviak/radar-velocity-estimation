function [ p_mc,o_mc ] = generateMicroClusters( P,idx,t_now,lambda,beta,mu,colors )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% NOTE:
% 1. DBSCAN outliers are assigned an idx value of zero and will not be
%    assigned to either p- or o-micro-clusters
% 2. For the initial scan, only p-micro-clusters should be generated, with
%    weight equal to sum(2^0) = N, the number of points in the cluster.

% UNFINISHED:
% 1. radius calculation from Definition 3.4 is yielding imaginary
%    numbers...
% 2. Use repmat() to init array of micro-cluster structs

% define fading function
fading = @(t) 2^(-lambda*(t_now - t));

%% BEGIN FUNCTION

% init micro-cluster struct - how to init array of empty structs?
% ANS: Use repmat - https://stackoverflow.com/questions/13664090/how-to-initialize-an-array-of-structs-in-matlab
p_mc = struct([]);
o_mc = struct([]);

% loop through each cluster identified by DBSCAN
for i=1:max(idx)
    cluster_idx = (idx == i);
    
    % assign points to new p-micro-cluster
    mc.points = P(cluster_idx,:);
    
    % init p-micro-cluster incremental parameters
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
    fprintf('\nmc(%d).radius =\n',i);
    mc.radius = sqrt(norm(mc.CF2) / mc.weight - ...
                    (norm(mc.CF1) / mc.weight)^2);
    disp(mc.radius)
    
    % radius measurement with switched terms
    mc.radius = sqrt((norm(mc.CF1) / mc.weight)^2 - ...
                      norm(mc.CF2) / mc.weight);
    disp(mc.radius)
                
    % unweighted radius measurement
    dist = pdist2(mc.center,mc.points(:,1:2),'euclidean');
    mc.radius = max(dist);
    disp(mc.radius)
                     
   if mc.weight >= beta*mu
       % add unique color to p-micro-cluster
       mc.color = colors(i,:);
       
       % assign micro-cluster to list of p-micro-clusters
       if isempty(p_mc)
           % create new array of p-micro-clusters
           p_mc = mc;
       else
           % append current micro-cluster to array of p-micro-clusters
           p_mc = [p_mc; mc];
       end 
   else
       % add initializtion time to o-micro-cluster
       mc.to = t_now;
       
       % assign micro-cluster to list of o-micro-clusters
       if isempty(o_mc)
           % create new array of o-micro-clusters
           o_mc = mc;
       else
           % append current micro-cluster to array of p-micro-clusters
           o_mc = [o_mc; mc];
       end
   end
    
end


end

