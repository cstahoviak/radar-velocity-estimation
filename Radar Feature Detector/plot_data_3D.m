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
filename = 'multiradar_2019-02-13_rangeRes_0-04_velRes_0-09_loop2x';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
bag_file = strcat(path,filename,'.bag');

% load(mat_file);
bag = rosbag(bag_file);

% list available frames
bag.AvailableFrames

%% Topic Information

radar_topic       = 'radar_fwd/mmWaveDataHdl/RScan';
vrpn_twist_topic  = '/vrpn_client_node/RadarQuad/twist';
vrpn_pose_topic   = '/vrpn_client_node/RadarQuad/pose';
odom_topic        = '/odom';
tf_topic          = '/tf';

%% Create Plotting Figure

fig1 = figure(1);
ax1 = axes('Parent',fig1);
grid on;
hold on;

%% Understand how to use ROS TFs

intensity_range = [6 35];

pose_bag    = select(bag,'Topic',vrpn_pose_topic);
pose_struct = readMessages(pose_bag,'DataFormat','struct');

% can extract data using cellfun()... new to me
pose_position_x = cellfun(@(m) double(m.Pose.Position.X),pose_struct);
pose_position_y = cellfun(@(m) double(m.Pose.Position.Y),pose_struct);
pose_position_z = cellfun(@(m) double(m.Pose.Position.Z),pose_struct);

radar_bag      = select(bag,'Topic',radar_topic);
radar_messages = readMessages(radar_bag);
radar_struct   = readMessages(radar_bag,'DataFormat','struct');

for i=1:size(radar_messages,1)
    
    fprintf('radar message %d', i)
  
    radar_x         = readField(radar_messages{i}, 'x');
    radar_y         = readField(radar_messages{i}, 'y');
    radar_z         = readField(radar_messages{i}, 'z');
    radar_range     = readField(radar_messages{i}, 'range');
    radar_intensity = readField(radar_messages{i}, 'intensity');

    % normalize intensity data for plotting
    int_norm = (radar_intensity - intensity_range(1))./ ...
        (intensity_range(2) - intensity_range(1));

    % loop through each traget in radar message
    for j=1:radar_struct{i}.Width
        % need to create PointStamped message, because only the following
        % message types can be transformed under the current Matlab/ROS
        % framework:
        
        % geometry_msgs/PointStamped
        % geometry_msgs/PoseStamped
        % geometry_msgs/PointCloud2Stamped
        % geometry_msgs/QuaternionStamped
        % geometry_msgs/Vector3Stamped
        
        % create PointStamped message to transform
        pt = rosmessage('geometry_msgs/PointStamped');
        pt.Header.FrameId = radar_struct{i}.Header.FrameId;
        pt.Header.Stamp = rostime(radar_struct{i}.Header.Stamp.Sec, ...
            radar_struct{i}.Header.Stamp.Nsec);
        pt.Point.X = radar_x(j);
        pt.Point.Y = radar_y(j);
        pt.Point.Z = radar_z(j);
%         disp([pt.Point.X, pt.Point.Y, pt.Point.Z])

        % transform point in 'radar_fwd_link' frame to 'world' frame
        tfTime = rostime(radar_struct{i}.Header.Stamp.Sec, ...
            radar_struct{i}.Header.Stamp.Nsec);
        if canTransform(bag,'world',radar_struct{i}.Header.FrameId,tfTime)
%             disp('can transform!')
            tf = getTransform(bag,'world',radar_struct{i}.Header.FrameId,tfTime);
            tfpt = apply(tf,pt);
%             disp(tfpt.Point)

            % plot transformed points
            scatter3(ax1,tfpt.Point.X,tfpt.Point.Y,tfpt.Point.Z,10, ... 
                'MarkerFaceColor','b', 'MarkerEdgeColor','b', ...
                'MarkerFaceAlpha',int_norm(j), 'MarkerEdgeAlpha',0);
        else
            if j==1
                fprintf('radar message %d: TF not available!', i)
            end
        end

    end
end

