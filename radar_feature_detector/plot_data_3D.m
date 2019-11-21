%% Header

% Filename:         plot_data_3D.m
% Author:           Carl Stahoviak
% Date Created:     09/13/2018
% Lasted Edited:    09/14/2019

% **Description here**

% ToDo:

clc;
clear;
close ALL;

%% Load Data and ROS bag

path     = '/home/carl/Data/subT/Fleming/multiradar_2019_02_13/';
device   = '1642';
filename = 'multiradar_2019-02-13_rangeRes_0-04_velRes_0-09_loop';
filetype = '.mat';

% path     = '/home/carl/Data/subT/Fleming/multiradar_2019_01_31/';
% device   = '1642';
% filename = 'multiradar_2019-01-31_cfar_5120_velMax_2.26_pkgrp_doppler_vy-neg';
% filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
bag_file = strcat(path,filename,'.bag');

% load(mat_file);
bag = rosbag(bag_file);

% list available frames
bag.AvailableFrames

%% Topic Information

radar_topic       = '/radar_fwd/mmWaveDataHdl/RScan';
vrpn_twist_topic  = '/vrpn_client_node/RadarQuad/twist';
vrpn_pose_topic   = '/vrpn_client_node/RadarQuad/pose';
odom_topic        = '/odom';
tf_topic          = '/tf';

%% Create Plotting Figure

fig1 = figure(1);
ax1 = axes('Parent',fig1);
grid on;
hold on;

intensity_range = [6 35];

%% Extract Radar Bag

radar_bag    = select(bag,'Topic',radar_topic);
radar_struct = readMessages(radar_bag,'DataFormat','struct');

% extract timestamp information (Christopher's method)
time_stamp_table = radar_bag.MessageList(:,1);
radar_time_stamp  = time_stamp_table{:,1};
radar_time_second = radar_time_stamp - radar_time_stamp(1);

%% Extract Pose Data

pose_bag    = select(bag,'Topic',vrpn_pose_topic);
pose_struct = readMessages(pose_bag,'DataFormat','struct');

% extract timestamp information (Christopher's method)
time_stamp_table = pose_bag.MessageList(:,1);
pose_time_stamp  = time_stamp_table{:,1};
pose_time_second = pose_time_stamp - pose_time_stamp(1);

% can extract data using cellfun()... new to me
pose_position_x = cellfun(@(m) double(m.Pose.Position.X),pose_struct);
pose_position_y = cellfun(@(m) double(m.Pose.Position.Y),pose_struct);
pose_position_z = cellfun(@(m) double(m.Pose.Position.Z),pose_struct);

orientation_x = cellfun(@(m) double(m.Pose.Orientation.X),pose_struct);
orientation_y = cellfun(@(m) double(m.Pose.Orientation.Y),pose_struct);
orientation_z = cellfun(@(m) double(m.Pose.Orientation.Z),pose_struct);
orientation_w = cellfun(@(m) double(m.Pose.Orientation.W),pose_struct);

%% Extract Radar Data

radar_bag      = select(bag,'Topic',radar_topic);
radar_messages = readMessages(radar_bag);
radar_struct   = readMessages(radar_bag,'DataFormat','struct');

% return;

%% Understand how to use ROS TFs

for i=1:size(radar_messages,1)
    
%     fprintf('radar message %d\n', i)
  
    radar_x         = readField(radar_messages{i}, 'x');
    radar_y         = readField(radar_messages{i}, 'y');
    radar_z         = readField(radar_messages{i}, 'z');
    radar_range     = readField(radar_messages{i}, 'range');
    radar_intensity = readField(radar_messages{i}, 'intensity');

    % normalize intensity data for plotting
    int_norm = (radar_intensity - intensity_range(1))./ ...
        (intensity_range(2) - intensity_range(1));
    
    % Only the following message types can be transformed under the
    % current Matlab/ROS framework:
        % geometry_msgs/PointStamped
        % geometry_msgs/PoseStamped
        % geometry_msgs/QuaternionStamped
        % geometry_msgs/Vector3Stamped
        % sensor_msgs/PointCloud2
    
    % transform point in 'radar_fwd_link' frame to 'world' frame
    tfTime = rostime(radar_struct{i}.Header.Stamp.Sec, ...
        radar_struct{i}.Header.Stamp.Nsec);
    
    % transform entire pcl rather than individual points!
    if canTransform(bag,'world',radar_struct{i}.Header.FrameId,tfTime)
%             disp('can transform!')
        tf = getTransform(bag,'world',radar_struct{i}.Header.FrameId,tfTime);
        radar_messages_tf = apply(tf,radar_messages{i});

        radar_x_tf = readField(radar_messages_tf, 'x');
        radar_y_tf = readField(radar_messages_tf, 'y');
        radar_z_tf = readField(radar_messages_tf, 'z');

        % plot transformed points
        % NOTE: cannot use 'MarkerFaceAlpha' to assign opacity to each
        % point in pcl based on intensity value. 'MarkerFaceAlpha' is a
        % scalar that applies to the entire pcl message.
%         scatter3(ax1,radar_x_tf,radar_y_tf,radar_z_tf,10, ... 
%             'MarkerFaceColor','b', 'MarkerEdgeColor','b', ...
%             'MarkerFaceAlpha',0.5, 'MarkerEdgeAlpha',0);

        % get index of closest Vicon velocity estimate to current time step
        [~,ix] = min( abs(pose_time_second - radar_time_second(i)) );
        
        x = pose_position_x(ix,1) + radar_x;
        y = pose_position_y(ix,1) + radar_y;
        z = pose_position_z(ix,1) + radar_z;
        
        scatter3(ax1, x, y, z, 10, ... 
            'MarkerFaceColor','b', 'MarkerEdgeColor','b', ...
            'MarkerFaceAlpha',0.5, 'MarkerEdgeAlpha',0);
    else
        fprintf('radar message %d: TF not available!\n', i)
    end
    
end

