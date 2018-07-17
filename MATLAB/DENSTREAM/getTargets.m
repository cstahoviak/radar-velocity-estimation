function [ P ] = getTargets( Ntargets_max,MU,SIGMA )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% random number of targets between 0 and Ntargets_max
Ntargets = randi(Ntargets_max);

% simulate radar targets {P}_i
t_now = toc;
allTargets = mvnrnd(MU,SIGMA);
P = [datasample(allTargets,Ntargets,'Replace',false), ...
     ones(Ntargets,1)*t_now];

end

