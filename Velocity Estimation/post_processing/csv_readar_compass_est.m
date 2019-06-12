%% Header

%%% Filename:   csv_reader_compass_est.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact

%% Load Data from CSV file

% Input file information
path     = '/home/carl/Data/subT/radar-rig/vicon_2019-05-08/processed_data/';
% path     = '/home/carl/Data/subT/Fleming/3d_velEstimation_2019-05-17/processed_data/';
subdir   = 'compass_ests/with_vel_005/';
run      = 'cfar-800_10Hz_run0_0/';
filename = 'compass_pose_aligned';
filetype = '.csv';

% Output file information
output_directory  = strcat(path,'mat_files/',subdir);
output_suffix     = '.mat';
output_filename   = strcat(output_directory,run(1:end-1),'_compass',output_suffix)

csv_file = strcat(path,subdir,run,filename,filetype)
data = csvread(csv_file);

%% Create variables to save

compass_time_stamp = data(:,1);
compass_time_second = data(:,1) - data(1,1);

compass_position_x = data(:,2);
compass_position_y = data(:,3);
compass_position_z = data(:,4);

compass_orientation_x = data(:,5);
compass_orientation_y = data(:,6);
compass_orientation_z = data(:,7);
compass_orientation_w = data(:,8);

compass_velocity_x = data(:,9);
compass_velocity_y = data(:,10);
compass_velocity_z = data(:,11);

%% Save Data to .mat file

save(output_filename, ...
    'compass_time_stamp','compass_time_second', ...
    'compass_position_x','compass_position_y','compass_position_z', ...
    'compass_orientation_x','compass_orientation_y','compass_orientation_z','compass_orientation_w', ...
    'compass_velocity_x','compass_velocity_y','compass_velocity_z');



