%% Header

%%% Filename:   test_angleBinning.m
%%% Author:     Carl Stahoviak
%%% Created:    04/03/2019 

clear;
clc;
close all;

format compact

%% Load Radar Data

path     = '/home/carl/Data/subT/Fleming/multiradar_2019-03-31_velocity_estimation/';
device   = '1642/';
filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

% undo RVIZ plotting defaults
radar_fwd_y = -radar_fwd_y;
radar_lat_y = -radar_lat_y;

% calculate radar angle
radar_fwd_angle = atan(radar_fwd_y./radar_fwd_x);   % [rad]
radar_lat_angle = atan(radar_lat_y./radar_lat_x);   % [rad]

%% Create Figures

figure(1)
ax1 = gca; hold on;
xlabel('time [s]','Interpreter','latex');
ylabel('angle [rad]','Interpreter','latex')
title('Forward Radar - Targets by Angle','Interpreter','latex')

figure(2)
ax2 = gca; hold on;
xlabel('time [s]','Interpreter','latex');
ylabel('angle [rad]','Interpreter','latex')
title('Lateral Radar - Targets by Angle','Interpreter','latex')

% polar plot
fh3 = figure(3);
ax3 = polaraxes('Parent',fh3);
hold on; grid on;
title('Point Target Location - Polar Coordinates','Interpreter','latex');
thetalim(ax3,[-135 135]);
ax3.ThetaZeroLocation = 'top';
ax3.ThetaDir = 'clockwise';

% polar plot
fh4 = figure(4);
ax4 = polaraxes('Parent',fh4);
hold on; grid on;
title('Point Target Location - Polar Coordinates','Interpreter','latex');
thetalim(ax4,[-135 135]);
ax4.ThetaZeroLocation = 'top';
ax4.ThetaDir = 'clockwise';

% load colors for plotting
load('colors.mat');
sz = 4;     % scatter plot marker size

%% Define Constants

% remove zero doppler velocity points?
filter_nonZero = false;

%% Main

NScans = size(radar_fwd_doppler,1);

% init angle bins
angle_bins_fwd = [];
angle_bins_lat = [];

for i=1:NScans
    
    % remove NaN and (possibly) zero-doppler values
    % NOTE: zero-doppler values have been shown to be artifacts of constant
    % intensity targets that are not representative of physical targets
    idx_pre_fwd = prefilter(radar_fwd_doppler(i,:), filter_nonZero);
    idx_pre_lat = prefilter(radar_lat_doppler(i,:), filter_nonZero);
    
    % plot targets by angular value
    n_fwd = size(radar_fwd_angle(i,idx_pre_fwd),2);
    time_fwd = ones(1,n_fwd)*radar_fwd_time_second(i);
    scatter(ax1, time_fwd, radar_fwd_angle(i,idx_pre_fwd), ...
        sz,colors(1,:),'filled');
    
    n_lat = size(radar_lat_angle(i,idx_pre_lat),2);
    time_lat = ones(1,n_lat)*radar_lat_time_second(i);
    scatter(ax2, time_lat, radar_lat_angle(i,idx_pre_lat), ...
        sz,colors(3,:),'filled');
    
    % polar plot
    polarscatter(ax3, radar_fwd_angle(i,idx_pre_fwd), ...
        radar_fwd_range(i,idx_pre_fwd),sz,colors(1,:),'filled');
    polarscatter(ax4, radar_lat_angle(i,idx_pre_lat), ...
        radar_lat_range(i,idx_pre_lat),sz,colors(3,:),'filled');
    
    % get unique angular locations in scan i
    angle_bins_temp_fwd = unique(radar_fwd_angle(i,idx_pre_fwd));
    angle_bins_temp_lat = unique(radar_lat_angle(i,idx_pre_lat));
    
    % update angle bin vector - forward radar
    angle_bins_fwd = [angle_bins_fwd; angle_bins_temp_fwd'];
    angle_bins_fwd = unique(angle_bins_fwd);
    
    % update angle bin vector - lateral radar
    angle_bins_lat = [angle_bins_lat; angle_bins_temp_lat'];
    angle_bins_lat = unique(angle_bins_lat);
    
end

%% Define Unique Angle Bins

bin_thres = 0.009;      % [rad]

% initialize
bins_fwd = [];
bins_lat = [];

current_bin = angle_bins_fwd(1);
begin_idx = 1;
update_plot = false;

for i=1:size(angle_bins_fwd,1)
    if abs(current_bin - angle_bins_fwd(i)) > bin_thres
        % update location of last angle to be averaged
        end_idx = i-1;
        
        % add single averaged value to table
        angle_bin = mean(angle_bins_fwd(begin_idx:end_idx));
        bins_fwd = [bins_fwd; angle_bin];
        
        % set new beginning index
        begin_idx = i;
        
        % set new current bin
        current_bin = angle_bins_fwd(i);
        
        update_plot = true;
        
    elseif i == size(angle_bins_fwd,1)
        % update location of last angle to be averaged
        end_idx = i;
        
        % add single averaged value to table
        angle_bin = mean(angle_bins_fwd(begin_idx:end_idx));
        bins_fwd = [bins_fwd; angle_bin];
        
        update_plot = true;
    end
    
    if update_plot
        % add data to plots
        plot(ax1,[0, radar_fwd_time_second(end)],[angle_bin, angle_bin], ...
            'Color',colors(2,:),'LineWidth',2)
        polarplot(ax3,angle_bin,10,'Color',colors(2,:))
        
        update_plot = false;
    end
    
    
end

current_bin = angle_bins_lat(1);
begin_idx = 1;
update_plot = false;

for i=1:size(angle_bins_lat,1)
    if abs(current_bin - angle_bins_lat(i)) > bin_thres
        % update location of last angle to be averaged
        end_idx = i-1;
        
        % add single averaged value to table
        angle_bin = mean(angle_bins_lat(begin_idx:end_idx));
        bins_lat = [bins_lat; angle_bin];
        
        % set new beginning index
        begin_idx = i;
        
        % set new current bin
        current_bin = angle_bins_lat(i);
        
        update_plot = true;
        
    elseif i == size(angle_bins_lat,1)
        % update location of last angle to be averaged
        end_idx = i;
        
        % add single averaged value to table
        angle_bin = mean(angle_bins_lat(begin_idx:end_idx));
        bins_lat = [bins_lat; angle_bin];
        
        update_plot = true;
    end
    
    if update_plot
        % add data to plots
        plot(ax2,[0, radar_lat_time_second(end)],[angle_bin, angle_bin], ...
            'Color',colors(2,:),'LineWidth',2)
        polarplot(ax4,angle_bin,10,'Color',colors(2,:))
        
        update_plot = false;
    end
        
end

% define radar angular bins
radar_angle_bins = mean([bins_fwd, bins_lat],2);

% calcualte sigma_theta by calcualting the angular bin spacing
diff = zeros(size(radar_angle_bins,1)-1,1);
for i=1:size(radar_angle_bins,1) - 1
    diff(i) = radar_angle_bins(i+1) - radar_angle_bins(i);
end

diff = [0; diff];
disp([bins_fwd, bins_lat, radar_angle_bins, diff])

% define sigma_theta as average angular bin spacing
sigma_theta = mean(diff(2:end))

%% Use getNumAngleBins()

[~,bins_fwd2] = getNumAngleBins(angle_bins_fwd')
[~,bins_lat2] = getNumAngleBins(angle_bins_lat')

disp([bins_fwd, bins_fwd2, bins_fwd - bins_fwd2, ...
    bins_lat, bins_lat2, bins_lat - bins_lat2]);






