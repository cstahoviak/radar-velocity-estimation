%% Header

%%% Filename:   csv_reader_gt.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact

%% Load Data from CSV file

% Input file information
path     = '/home/carl/Data/subT/Fleming/3d_velEstimation_2019-05-17/processed_data/';
run      = 'cfar-1280_10Hz_run2/';
filename = 'gt';
filetype = '.csv';

% Output file information
output_directory  = strcat(path,'mat_files/ground_truth/');
output_suffix     = '.mat';
output_filename   = strcat(output_directory,run(1:end-1),'_',filename,output_suffix)

csv_file = strcat(path,run,filename,filetype)
data = csvread(csv_file);

%% Create variables to save

gt_time_stamp = data(:,1);
gt_time_second = data(:,1) - data(1,1);

gt_position_x = data(:,2);
gt_position_y = data(:,3);
gt_position_z = data(:,4);

gt_orientation_x = data(:,5);
gt_orientation_y = data(:,6);
gt_orientation_z = data(:,7);
gt_orientation_w = data(:,8);

gt_velocity_x = data(:,9);
gt_velocity_y = data(:,10);
gt_velocity_z = data(:,11);

%% Save Data to .mat file

save(output_filename, ...
    'gt_time_stamp','gt_time_second', ...
    'gt_position_x','gt_position_y','gt_position_z', ...
    'gt_orientation_x','gt_orientation_y','gt_orientation_z','gt_orientation_w', ...
    'gt_velocity_x','gt_velocity_y','gt_velocity_z');



