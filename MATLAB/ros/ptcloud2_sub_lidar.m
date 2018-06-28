clear;
clc;
close all;

% init lidar subscriber
ldr_sub = rossubscriber('/pc2');

% define lidar data figure
fig1 = figure(1);
ax1 = axes('Parent',fig1);

% ldr_pc2 = receive(ldr_sub,10);
% return;

while true
    %%% get lidar pointcloud
    ldr_pc2 = receive(ldr_sub,10);
    
    ldr_xyz = readXYZ(ldr_pc2);
    ldr_x = ldr_xyz(:,1);
    ldr_y = ldr_xyz(:,2);
    
    % lidar intensity data not available
%     ldr_intensity = readField(ldr_pc2, 'intensity');
%     ldr_intensity = (ldr_intensity./100);
    
    for(i=1:length(ldr_x))
        scatter(ax1,ldr_x(i),ldr_y(i),10,'r','filled')
        xlim(ax1,[-15,15]); ylim(ax1,[-15,15]);
        hold on;
    end
    hold off;
    
end