clear;
clc;
close all;

% init radar subscriber
rdr_sub = rossubscriber('/mmWaveDataHdl/RScan');

% define radar data figure
fig1 = figure(1);
ax1 = axes('Parent',fig1);

% rdr_pc2 = receive(rdr_sub,10);
% return;

while true    
    %%% get lidar pointcloud
    rdr_pc2 = receive(rdr_sub,10);
    
    rdr_xyz = readXYZ(rdr_pc2);
    rdr_x = rdr_xyz(:,1);
    rdr_y = rdr_xyz(:,2);
    
    % note: lidar intensity data not available
    rdr_intensity = readField(rdr_pc2, 'intensity');
    rdr_intensity = (rdr_intensity./100);
    
    for(i=1:length(rdr_x))
        scatter(ax1,rdr_x(i),rdr_y(i),'MarkerFaceColor','b',...
            'MarkerEdgeColor','b','MarkerFaceAlpha',rdr_intensity(i),...
            'MarkerEdgeAlpha',0)
        xlim(ax1,[0,15]); ylim(ax1,[-15,15]);
        hold on;
    end
    hold off;
    
end