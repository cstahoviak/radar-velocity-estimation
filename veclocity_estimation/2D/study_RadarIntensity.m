%% Header

%%% Filename:   study_RadarIntensity.m
%%% Author:     Carl Stahoviak
%%% Created:    04/03/2019 

clear;
clc;
close all;

format compact

%% Load Data

path     = '/home/carl/Data/subT/Fleming/multiradar_2019-03-31_velocity_estimation/';
device   = '1642/';
filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

%% Modify Radar Data

radar_range       = radar_fwd_range;
radar_intensity   = radar_fwd_intensity;
radar_doppler     = radar_fwd_doppler;
radar_time_second = radar_fwd_time_second;

radar_x = radar_fwd_x;
radar_y = radar_fwd_y;

% undo RVIZ plotting defaults
radar_y = -radar_y;

% calculate radar angle
radar_angle = atan(radar_y./radar_x);   % [rad]

%% Create Figures

% Point target intensity figure - Zero Doppler
fig1 = figure(1);
ax1 = axes('Parent',fig1);
hold on; grid on;
title('Point Target Intensity Value - Zero Doppler','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Intensity [dB]','Interpreter','latex');
xlim([0 radar_time_second(end)]);

% Point target intensity figure - Range Filtering
fig2 = figure(2);
ax2 = axes('Parent',fig2);
hold on; grid on;
title('Point Target Intensity Value - Range Filtering','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Intensity [dB]','Interpreter','latex');
xlim([0 radar_time_second(end)]);

sz = 4;

%% Define Constants

% set filtering thresholds
range_thres     = 0.30;     % [m]
angle_thres     = 90;       % [deg]
intensity_thres = 5;        % [dB]
thresholds_AIR  = [angle_thres; intensity_thres; range_thres];

% remove zero doppler velocity points?
filter_nonZero = false;

%% Main

NScans = size(radar_doppler,1);

for i=1:NScans
    
    % remove NaN and (possibly) zero-doppler values
    % NOTE: zero-doppler values have been shown to be artifacts of constant
    % intensity targets that are not representative of physical targets
    idx_pre = prefilter(radar_doppler(i,:), filter_nonZero);
    Ntargets = sum(idx_pre);    % number of valid targets per scan
    
    % extract index of points with zero (or non-zero) Doppler velocity
    idx_zero = (radar_doppler(i,:) == 0);
    idx_nonZero = (radar_doppler(i,:) ~= 0);
    
    % define time vector for scatter plotting
    t = radar_time_second(i)*ones(sum(idx_pre & idx_nonZero),1);
    t_isZero = radar_time_second(i)*ones(sum(idx_pre & idx_zero),1);
    
    % plot (non-zero doppler) intensity values at each time step
    h1_1 = scatter(ax1,t,radar_intensity(i,idx_pre & idx_nonZero), ...
        sz,'b','filled');
    % filter intensity by zero Doppler velocity, NOTE: interesting
    % horizantal line trend!
    h1_2 = scatter(ax1,t_isZero,radar_intensity(i,idx_pre & idx_zero), ...
        sz,'r','filled');
    
    % apply {angle, intensity, range} 'AIR' filtering
    data_AIR = [radar_angle(i,idx_pre)', radar_intensity(i,idx_pre)', ...
        radar_range(i,idx_pre)'];
    [ idx_AIR ] = AIR_filtering( data_AIR, thresholds_AIR);
    
    % can I use range filtering to match the zero-doppler filtering
    intensity = radar_intensity(i,idx_pre);
    
    % define time vector for scatter plotting
    t1 = radar_time_second(i)*ones(sum(idx_AIR),1);
    t2 = radar_time_second(i)*ones(sum(~idx_AIR),1);
    
    h2_1 = scatter(ax2,t1,intensity(idx_AIR),sz,'b','filled');
    h2_2 = scatter(ax2,t2,intensity(~idx_AIR),sz,'r','filled');
    
    
end