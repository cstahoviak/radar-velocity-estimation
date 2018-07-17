%% Header

% Filename:         denstream_main.m
% Author:           Carl Stahoviak
% Date Created:     07/11/2018
% Lasted Edited:    07/17/2018

% **Description here**

% ToDo:
% 1. Change how targets are simulated. Place Some number of targets in the
%   scene with their own mean and std dev. Then pull randomly from these
%   distributions for successive scans to me merged with map.

clc;
clear;
close ALL;

%% DENSTREAM PARAMETERS

lambda    = 2;      % fading factor
eps       = 0.8;    % radius of epsilon-neighborhood
minPts    = 2;      % minimum number of points in a cluster, DBSCAN parameter
beta      = 0.001;    % parameter to distinguish between relative weights of p- and o-micro-clusters
mu        = 3;      % core micro-cluster weught, integer value
Tp        = 3;      % checking period for cluster weights, i.e. "pruning" period

% Equation 4.1, DENSTREAM paper
% Tp = (-1/lambda)*log2(1 - 1/(beta*mu));

distType = 'euclid';

%% ADDITIONAL PARAMETERS

% [x y peakVal] min/max values
MAX = [ 10 50 5000];
MIN = [-10 0  0];

% assign unique color to each p-micro-cluster
colors = hsv(25);   % hsv(N) assumes a max of N p-micro-clusters

% define wait time after new data has been populated to plot
waitTime = 2;

% create plotting figure
figure(1);
ax = gca;

%% INITIALIZATION

% start system stopwatch
tic;

% maximum number of targets per scan
Ntargets_max = 25;

% generate target statistics
% load('target_stats.mat')
MU = zeros(Ntargets_max,3);
for i=1:Ntargets_max
    MU(i,:) = [(MAX(1) - MIN(1))*rand() + MIN(1), ...
               (MAX(2) - MIN(2))*rand() + MIN(2), ...
               (MAX(3) - MIN(3))*rand() + MIN(3)];
end

SIGMA = [0.5 0   0;
         0   0.5 0;
         0   0   0.1];

% get initial set of points (targets), with at least one cluster as
% identified by DBSCAN using the {eps,minPts} parameters
[ P,idx ] = denstream_init( ax,Ntargets_max,MU,SIGMA,eps,minPts, ...
               MIN,MAX,waitTime,colors );

% generate initial set of p-micro-clusters
% NOTE: no o-micro-clusters generated from initial scan
t_now = toc;
[ p_mc,o_mc ] = generateMicroClusters( P,idx,t_now,lambda,beta,mu,colors )

% update plot of micro-clusters
updateClusterPlot( ax,MIN,MAX,p_mc,o_mc );
pause(waitTime);

return;

%% MAIN LOOP

% total number of radar scans to be merged with map via DENSTREAM
Nscans = 5;

count = 1;
while(count <= Nscans)
    fprintf('p-mc: %d\t o-mc: %d\n',length(p_mc),length(o_mc));

    % simulate radar targets {P}_i
    [ P ] = getTargets( Ntargets_max,MU,SIGMA );
    
    % plot points in new scan with black 'x'
    scatter(ax,P(:,1),P(:,2),15,'kx'); hold off;
    xlim([MIN(1)-5 MAX(1)+5]); ylim([MIN(2)-5 MAX(2)+5]);
    pause(waitTime);
    
    % pass {P}_i to DENSTREAM()
    [ p_mc,o_mc ] = DENSTREAM( P,p_mc,o_mc,lambda,eps,beta,mu,Tp, ...
                       distType,t_now,colors );
    
    % update plot of micro-clusters
    updateClusterPlot( ax,MIN,MAX,p_mc,o_mc );
    pause(waitTime);
    
    count = count+1;
end
