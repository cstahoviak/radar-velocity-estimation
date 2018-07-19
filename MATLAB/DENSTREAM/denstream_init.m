function [ P,idx ] = denstream_init( ax,Ntargets_max,MU,SIGMA,eps,minPts,MIN,MAX,waitTime,colors )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% init array of targets
P = [];

idx = zeros(Ntargets_max,1);
while sum(idx) == 0    
    % simulate radar targets {P}_i
    [ targets ] = getTargets( Ntargets_max,MU,SIGMA );
    P = [P; targets];

    % plot initial points before DBSCAN clustering
    scatter(ax,P(:,1),P(:,2),15,'kx'); grid on;
    xlim([MIN(1)-5 MAX(1)+5]); ylim([MIN(2)-5 MAX(2)+5]);
    pause(waitTime);

    % Apply DBSCAN to the first set of points {P} to initilize the online process
    idx = DBSCAN( P(:,1:2),eps,minPts );
end
% hold on;

% plot DBSCAN results - does this assign the same colors to the
% p-micro-clusters that I assign to them in generateMicroClusters()??
PlotDBSCANResult(ax,P(:,1:2),idx,colors);
grid on; hold on;
title(['DBSCAN Clustering (\epsilon = ' num2str(eps) ', minPts = ' num2str(minPts) ')']);
xlim([MIN(1)-5 MAX(1)+5]); ylim([MIN(2)-5 MAX(2)+5]);
pause(waitTime);

end
