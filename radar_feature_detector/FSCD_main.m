%% Header

% Filename:         FSCD_main.m
% Author:           Carl Stahoviak
% Date Created:     08/14/2018
% Lasted Edited:    02/12/2019

% **Description here**

% ToDo:

clc;
clear;
close ALL;

%% Load Data and ROS bag

path     = '/home/carl/Data/subT/Fleming/multiradar_2019_02_13/';
device   = '1642';
filename = 'multiradar_2019-02-13_rangeRes_0-04_velRes_0-09_loop2x';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
bag_file = strcat(path,filename,'.bag');

% load(mat_file);
bag = rosbag(bag_file);

% list available frames
bag.AvailableFrames


%% DEFINE CONSTANTS

NScans = 10;            % number of radar scans aggregated into a measurment grid
range_res = 0.044;      % radar range resolution [m]
doppler_res = 0.13;     % radar doppler resolution [m/s]

% topic information
radar_topic       = 'radar_fwd/mmWaveDataHdl/RScan';
vrpn_twist_topic  = '/vrpn_client_node/RadarQuad/twist';
vrpn_pose_topic   = '/vrpn_client_node/RadarQuad/pose';
odom_topic        = '/odom';
tf_topic          = '/tf';
   

%% read incoming radar target data

for i=1:NScans:size(radar_x,1)
    
    % init empty array for radar data for the next NScans
    radar_data = [];
    
    for j=0:NScans-1
        
        % get pose data from Vicon system - handles non-uniform
        % sample rates between radar and Vicon system
         [~,idx] = min( abs ( pose_time_second - radar_time_second(i+j) ) );

        % get all non-NaN targets from radar scan
        % NOTE: may also want to remove zero doppler targets
        idx_nonNaN = ~isnan(radar_x(i+j,:));
        rdr_x = radar_x(i+j,idx_nonNaN);
        rdr_y = radar_y(i+j,idx_nonNaN);
        intensity = radar_intensity(i+j,idx_nonNaN);
        
        % place radar measurements into global frame
        x = rdr_x + pose_position_x(idx,1);
        y = rdr_y + pose_position_y(idx,1);
        
        % append current scan to radar_data
        current_scan = [x', y', intensity'];
        if isempty(radar_data)
            radar_data = current_scan;
        else
            radar_data = [radar_data; current_scan];
        end   
    end
    
    % get dimensions of measurement grid
    min_x = min(min(radar_data(:,1)));
    max_x = max(max(radar_data(:,1)));
    min_y = min(min(radar_data(:,2)));
    max_y = max(max(radar_data(:,2)));
    
    % create measurment grid
    meas_grid = zeros((max_y - min_y)/range_res, (max_x - min_x)/range_res);
    
    % for each target in radar_data add intensity data to meas. grid
    for i=1:size(radar_data,1)
        
        row = radar_data(i,2)/range_res;
        col = radar_data(i,1)/range_res;
        
        
    end
    
    % place radar_data into 

end



