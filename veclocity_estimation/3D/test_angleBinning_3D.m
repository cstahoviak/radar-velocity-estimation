%% Header

%%% Filename:   test_angleBinning_3D.m
%%% Author:     Carl Stahoviak
%%% Created:    04/03/2019 

clear;
clc;
close all;

format compact

%% Load Radar Data

path     = '/home/carl/Data/subT/Fleming/fleming_radar_2019-05-17/';
filename = 'cfar-800_10Hz_run0';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

% undo RVIZ plotting defaults
radar_y = -radar_y;
radar_z = -radar_z;

% calculate radar angle
radar_azimuth = atan(radar_y./radar_x);         % [rad]
radar_elevation = asin(radar_z./radar_range);   % [rad]

%% Create Figures

figure(1)
ax1 = gca; hold on;
xlabel('time [s]','Interpreter','latex');
ylabel('azimuth [rad]','Interpreter','latex')
title('Targets by Azimuth','Interpreter','latex')

figure(2)
ax2 = gca; hold on;
xlabel('time [s]','Interpreter','latex');
ylabel('elevation [rad]','Interpreter','latex')
title('Targets by Elevation','Interpreter','latex')

% polar plot
fh3 = figure(3);
ax3 = polaraxes('Parent',fh3);
hold on; grid on;
title('Point Target Azimuth','Interpreter','latex');
thetalim(ax3,[-135 135]);
ax3.ThetaZeroLocation = 'top';
ax3.ThetaDir = 'clockwise';

% polar plot
fh4 = figure(4);
ax4 = polaraxes('Parent',fh4);
hold on; grid on;
title('Point Target Elevation','Interpreter','latex');
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

NScans = size(radar_doppler,1);

% init angle bins
azimuth_bins = [];
elevation_bins = [];

for i=1:NScans
    
    % remove NaN and (possibly) zero-doppler values
    % NOTE: zero-doppler values have been shown to be artifacts of constant
    % intensity targets that are not representative of physical targets
    idx_pre = prefilter(radar_doppler(i,:), filter_nonZero);
    
    % plot targets by azimuth value
    n = size(radar_azimuth(i,idx_pre),2);
    time = ones(1,n)*radar_time_second(i);
    scatter(ax1, time, radar_azimuth(i,idx_pre), ...
        sz,colors(1,:),'filled');
    
    % plot targets by elevation value
    scatter(ax2, time, radar_elevation(i,idx_pre), ...
        sz,colors(1,:),'filled');
    
    % polar plot
    polarscatter(ax3, radar_azimuth(i,idx_pre), ...
        radar_range(i,idx_pre),sz,colors(1,:),'filled');
    polarscatter(ax4, radar_elevation(i,idx_pre), ...
        radar_range(i,idx_pre),sz,colors(3,:),'filled');
    
    % get unique angular locations in scan i
    az_bins_temp = unique(radar_azimuth(i,idx_pre));
    elev_bins_temp = unique(radar_elevation(i,idx_pre));
    
    % update azimuth bin vector
    azimuth_bins = [azimuth_bins; az_bins_temp'];
    azimuth_bins = unique(azimuth_bins);
    
    % update elevation bin vector
    elevation_bins = [elevation_bins; elev_bins_temp'];
    elevation_bins = unique(elevation_bins);
    
end

%% Define Unique Angle Bins using getNumAngleBins()

[~,bins_az] = getNumAngleBins(azimuth_bins');
[~,bins_elev] = getNumAngleBins(elevation_bins');

% calculate sigma_theta by calculating the angular bin spacing
diff_az = zeros(size(bins_az,1)-1,1);
for i=1:size(bins_az,1) - 1
    diff_az(i) = azimuth_bins(i+1) - azimuth_bins(i);
end

% calculate sigma_phi by calculating the angular bin spacing
diff_elev = zeros(size(bins_elev,1)-1,1);
for i=1:size(bins_elev,1) - 1
    diff_elev(i) = elevation_bins(i+1) - elevation_bins(i);
end

% diff = [0; diff];
% disp([bins_fwd, bins_lat, radar_angle_bins, diff])

% define sigma_theta as average angular bin spacing
sigma_theta = mean(diff_az(2:end))
sigma_phi = mean(diff_elev(2:end))






