%% Header

%%% Filename:   post.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact

%% Load Vicon System Data

% vehicle = 'jackal';
vehicle = 'quad';

if strcmp(vehicle,'quad')
    path = '/home/carl/Data/subT/Fleming/3d_velEstimation_2019-05-17/';
elseif strcmp(vehicle,'jackal')
    path = '/home/carl/Data/subT/radar-rig/vicon_2019-05-08/';
else
    path = '';
end
filename = 'cfar-800_10Hz_run0';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

% modify pose position data - start at (0,0,0)
pose_position_x = pose_position_x - pose_position_x(1);
pose_position_y = pose_position_y - pose_position_y(1);
pose_position_z = pose_position_z - pose_position_z(1);

% NOTE: Vicon pose message orientation in N.W.U. frame
orientation = [pose_orientation_w, ... 
               pose_orientation_x, ... 
               pose_orientation_y, ...
               pose_orientation_z];

twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
twist_linear_body = getBodyFrameVelocities( twist_linear, orientation, ...
    pose_time_stamp, twist_time_stamp );

% Map from NWU to NED coordinate frame
twist_linear_body(:,2) = -twist_linear_body(:,2);
twist_linear_body(:,3) = -twist_linear_body(:,3);

%% Load Andrew's Ground Truth Data

gt_path = strcat(path,'processed_data/mat_files/');
subdir   = 'ground_truth/';
suffix   = '_gt';

gt_file = strcat(gt_path,subdir,filename,suffix,filetype);
load(gt_file);

% modify pose position data - start at (0,0,0)
gt_position_x = gt_position_x - gt_position_x(1);
gt_position_y = gt_position_y - gt_position_y(1);
gt_position_z = gt_position_z - gt_position_z(1);

gt_position    = [gt_position_y, -gt_position_x, gt_position_z];
gt_orientation = [gt_orientation_w, gt_orientation_x, ...
                  gt_orientation_y, gt_orientation_z];
gt_velocity    = [gt_velocity_x, gt_velocity_y, gt_velocity_z];

%% Load Compass Data - WITH Radar Velocity (VID-SLAM)

subdir = 'compass_ests/with_vel/';
suffix = '_compass';

mat_file = strcat(gt_path,subdir,filename,suffix,filetype);
load(mat_file);

VID_time_stamp = compass_time_stamp;
VID_time_second = compass_time_second;

VID_position_x = compass_position_x;
VID_position_y = compass_position_y;
VID_position_z = compass_position_z;

VID_orientation_w = compass_orientation_w;
VID_orientation_x = compass_orientation_x;
VID_orientation_y = compass_orientation_y;
VID_orientation_z = compass_orientation_z;

VID_velocity_x = compass_velocity_x;
VID_velocity_y = compass_velocity_y;
VID_velocity_z = compass_velocity_z;

VID_position    = [VID_position_x, VID_position_y, VID_position_z];
VID_orientation = [VID_orientation_w, VID_orientation_x, ...
                       VID_orientation_y, VID_orientation_z];
VID_velocity    = [VID_velocity_x, VID_velocity_y, VID_velocity_z];

%% Load Compass Data - No Radar Velocity

subdir = 'compass_ests/no_vel/';
suffix = '_compass';

mat_file = strcat(gt_path,subdir,filename,suffix,filetype);
load(mat_file);

compass_position    = [compass_position_x, compass_position_y, compass_position_z];
compass_orientation = [compass_orientation_w, compass_orientation_x, ...
                       compass_orientation_y, compass_orientation_z];
compass_velocity    = [compass_velocity_x, compass_velocity_y, compass_velocity_z];

%% Create Body-Frame Velocity Ground Truth Data

% load colors for plotting
load('colors.mat');
sz = 4;                 % scatter plot marker size

% transfrom grouth-truth velocities
gt_velocity_body = getBodyFrameVelocities( gt_velocity, gt_orientation, ...
    gt_time_stamp, gt_time_stamp );

% transfrom VID velocities
VID_velocity_body = getBodyFrameVelocities( VID_velocity, VID_orientation, ...
    VID_time_stamp, VID_time_stamp );

% transfrom Compass velocities
compass_velocity_body = getBodyFrameVelocities( compass_velocity, ...
    compass_orientation, compass_time_stamp, compass_time_stamp );

% Map from NWU to NED coordinate frame
% gt_velocity_body(:,2) = -gt_velocity_body(:,2);
% gt_velocity_body(:,3) = -gt_velocity_body(:,3);

% get euler angles
euler_gt = quat2eul(gt_orientation);
euler_VID = quat2eul(VID_orientation);
euler_compass = quat2eul(compass_orientation);

%% Plot Results - Relative Time

% plot ground-truth body-frame velocites
figure(1)
subplot(3,1,1);
plot(twist_time_second,twist_linear_body(:,1)); hold on
plot(gt_time_second,gt_velocity_body(:,1)); 
title('Ground Truth Body-Frame Velocities, N.E.D.','Interpreter','latex');
ylabel('$v_x$ [m/s]','Interpreter','latex');
subplot(3,1,2);
plot(twist_time_second,twist_linear_body(:,2)); hold on
plot(gt_time_second,gt_velocity_body(:,2)); 
ylabel('$v_y$ [m/s]','Interpreter','latex');
subplot(3,1,3);
plot(twist_time_second,twist_linear_body(:,3)); hold on
plot(gt_time_second,gt_velocity_body(:,3)); 
ylabel('$v_z$ [m/s]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('Vicon system','Compass GT');
set(hdl,'Interpreter','latex')

% plot ground-truth position
figure(2)
subplot(3,1,1);
plot(pose_time_second,pose_position_x); hold on
plot(gt_time_second,gt_position(:,1)); 
title('Ground Truth Position','Interpreter','latex');
ylabel('$x$ [m]','Interpreter','latex');
subplot(3,1,2);
plot(pose_time_second,pose_position_y); hold on;
plot(gt_time_second,gt_position(:,2));
ylabel('$y$ [m]','Interpreter','latex');
subplot(3,1,3);
plot(pose_time_second,pose_position_z); hold on;
plot(gt_time_second,gt_position(:,3)); 
ylabel('$z$ [m]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('Vicon system','Compass GT');
set(hdl,'Interpreter','latex')

%% Plot Results - Time Stamp

% plot ground-truth body-frame velocites
figure(3)
subplot(3,1,1);
plot(twist_time_stamp,twist_linear_body(:,1)); hold on
plot(gt_time_stamp,gt_velocity_body(:,1)); 
title('Ground Truth Body-Frame Velocities, N.E.D.','Interpreter','latex');
ylabel('$v_x$ [m/s]','Interpreter','latex');
subplot(3,1,2); 
plot(twist_time_stamp,twist_linear_body(:,2)); hold on;
plot(gt_time_stamp,gt_velocity_body(:,2));
ylabel('$v_y$ [m/s]','Interpreter','latex');
subplot(3,1,3);
plot(twist_time_stamp,twist_linear_body(:,3)); hold on;
plot(gt_time_stamp,gt_velocity_body(:,3)); 
ylabel('$v_z$ [m/s]','Interpreter','latex');
% xlim([0, gt_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('Vicon system','Compass GT');
set(hdl,'Interpreter','latex')

% plot ground-truth position
figure(4)
subplot(3,1,1);
plot(pose_time_stamp,pose_position_x); hold on
plot(gt_time_stamp,gt_position(:,1));
title('Ground Truth Position','Interpreter','latex');
ylabel('$x$ [m]','Interpreter','latex');
subplot(3,1,2);
plot(pose_time_stamp,pose_position_y); hold on;
plot(gt_time_stamp,gt_position(:,2));
ylabel('$y$ [m]','Interpreter','latex');
subplot(3,1,3);
plot(pose_time_stamp,pose_position_z); hold on;
plot(gt_time_stamp,gt_position(:,3));
ylabel('$z$ [m]','Interpreter','latex');
% xlim([0, gt_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('Vicon system','Compass GT');
set(hdl,'Interpreter','latex')




