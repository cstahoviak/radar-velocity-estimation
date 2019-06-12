%% Header

%%% Filename:   post.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact

%% Load Vicon System Data

vehicle = 'jackal';
% vehicle = 'quad';

if strcmp(vehicle,'quad')
    path = '/home/carl/Data/subT/Fleming/3d_velEstimation_2019-05-17/';
elseif strcmp(vehicle,'jackal')
    path = '/home/carl/Data/subT/radar-rig/vicon_2019-05-08/';
else
    path = '';
end
filename = 'cfar-800_10Hz_run0';
subrun = '_0';
filetype = '.mat';

vicon_file = strcat(path,'mat_files/',filename,filetype)
load(vicon_file);

% modify pose position data - start at (0,0,0)
pose_position_x = pose_position_x - pose_position_x(1);
pose_position_y = pose_position_y - pose_position_y(1);
pose_position_z = pose_position_z - pose_position_z(1);

% NOTE: Vicon pose message orientation in N.W.U. frame
orientation = [pose_orientation_w, ... 
               pose_orientation_x, ... 
               pose_orientation_y, ...
               pose_orientation_z];

% twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
% twist_linear_body = getBodyFrameVelocities( twist_linear, orientation, ...
%     pose_time_stamp, twist_time_stamp );
% 
% % Map from NWU to NED coordinate frame
% twist_linear_body(:,2) = -twist_linear_body(:,2);
% twist_linear_body(:,3) = -twist_linear_body(:,3);

%% Ground Truth Vicon Velocity
% use central difference method to calculate derivative
h = 0.01;   % vrpn system @ 100 Hz
velocity_x = central_diff(pose_position_x, h);
velocity_y = central_diff(pose_position_y, h);
velocity_z = central_diff(pose_position_z, h);
velocity = [velocity_x, velocity_y, velocity_z];

velocity_time_stamp = pose_time_stamp(2:end-1);
velocity_time_second = pose_time_second(2:end-1);

% transform from Vicon-frame to body-frame
velocity_body = getBodyFrameVelocities( velocity, orientation, ...
        pose_time_stamp, velocity_time_stamp );
    
% Map from NWU to NED coordinate frame
velocity_body(:,2) = -velocity_body(:,2);
velocity_body(:,3) = -velocity_body(:,3);

sample_freq = 100;     % vrpn system @ 100 Hz
fpass = 0.12;
velocity_body = lowpass(velocity_body,fpass,sample_freq);

% implement moving-average filtering
span = 5;
method = 'moving';

smoothed_vx = smooth(velocity_body(:,1),span,method);
smoothed_vy = smooth(velocity_body(:,2),span,method);
smoothed_vz = smooth(velocity_body(:,3),span,method);

smoothed_velocity_body = [smoothed_vx, smoothed_vy, smoothed_vz];

%% Load Andrew's Ground Truth Data

gt_path = strcat(path,'processed_data/mat_files/');
subdir   = 'ground_truth/';
suffix   = '_gt';

gt_file = strcat(gt_path,subdir,filename,suffix,filetype)
load(gt_file);

% modify pose position data - start at (0,0,0)
% gt_position_x = gt_position_x - gt_position_x(1);
% gt_position_y = gt_position_y - gt_position_y(1);
% gt_position_z = gt_position_z - gt_position_z(1);

gt_position    = [gt_position_x, gt_position_y, gt_position_z];
gt_orientation = [gt_orientation_w, gt_orientation_x, ...
                  gt_orientation_y, gt_orientation_z];
gt_velocity    = [gt_velocity_x, gt_velocity_y, gt_velocity_z];

%% Load Compass Data - WITH Radar Velocity (VID-SLAM)

subdir = 'compass_ests/with_vel/';
if strcmp(vehicle,'jackal')
%     suffix = strcat(subrun,'_compass');
    suffix = strcat('_compass');
elseif strcmp(vehicle,'quad')
    suffix = '_compass';
else
    suffix = '';
end

with_vel_file = strcat(gt_path,subdir,filename,suffix,filetype)
load(with_vel_file);

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

no_vel_file = strcat(gt_path,subdir,filename,suffix,filetype)
load(no_vel_file);

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

%% Create Speed Data

gt_speed = sqrt( sum( (gt_velocity.^2),2 ) );
VID_speed = sqrt( sum( (VID_velocity.^2),2 ) );
compass_speed = sqrt( sum( (compass_velocity.^2),2 ) );

%% Plot Results - Relative Time

% % plot ground-truth body-frame velocites
% figure(1)
% subplot(3,1,1);
% plot(velocity_time_second,smoothed_velocity_body(:,1)); hold on
% plot(gt_time_second,gt_velocity_body(:,1)); 
% title('Ground Truth Body-Frame Velocities, N.E.D.','Interpreter','latex');
% ylabel('$v_x$ [m/s]','Interpreter','latex');
% subplot(3,1,2);
% plot(velocity_time_second,smoothed_velocity_body(:,2)); hold on
% plot(gt_time_second,gt_velocity_body(:,2)); 
% ylabel('$v_y$ [m/s]','Interpreter','latex');
% subplot(3,1,3);
% plot(velocity_time_second,smoothed_velocity_body(:,3)); hold on
% plot(gt_time_second,gt_velocity_body(:,3)); 
% ylabel('$v_z$ [m/s]','Interpreter','latex');
% xlim([0, gt_time_second(end)]);
% xlabel('time [s]','Interpreter','latex');
% hdl = legend('Vicon system','Compass GT');
% set(hdl,'Interpreter','latex')
% 
% % plot ground-truth position
% figure(2)
% subplot(3,1,1);
% plot(pose_time_second,pose_position_x); hold on
% plot(gt_time_second,gt_position(:,1)); 
% title('Ground Truth Position','Interpreter','latex');
% ylabel('$x$ [m]','Interpreter','latex');
% subplot(3,1,2);
% plot(pose_time_second,pose_position_y); hold on;
% plot(gt_time_second,gt_position(:,2));
% ylabel('$y$ [m]','Interpreter','latex');
% subplot(3,1,3);
% plot(pose_time_second,pose_position_z); hold on;
% plot(gt_time_second,gt_position(:,3)); 
% ylabel('$z$ [m]','Interpreter','latex');
% xlim([0, gt_time_second(end)]);
% xlabel('time [s]','Interpreter','latex');
% hdl = legend('Vicon system','Compass GT');
% set(hdl,'Interpreter','latex')

%% Plot Results - Time Stamp

% plot ground-truth body-frame velocites
figure(3)
subplot(3,1,1);
plot(velocity_time_stamp,smoothed_velocity_body(:,1)); hold on
plot(gt_time_stamp,gt_velocity_body(:,1)); 
title('Ground Truth Body-Frame Velocities, N.E.D.','Interpreter','latex');
ylabel('$v_x$ [m/s]','Interpreter','latex');
subplot(3,1,2); 
plot(velocity_time_stamp,smoothed_velocity_body(:,2)); hold on;
plot(gt_time_stamp,gt_velocity_body(:,2));
ylabel('$v_y$ [m/s]','Interpreter','latex');
subplot(3,1,3);
plot(velocity_time_stamp,smoothed_velocity_body(:,3)); hold on;
plot(gt_time_stamp,gt_velocity_body(:,3)); 
ylabel('$v_z$ [m/s]','Interpreter','latex');
% xlim([0, gt_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('Vicon system','Compass GT');
set(hdl,'Interpreter','latex')

% plot ground-truth position - Compass and Vicon do NOT share the same
% reference frame -> this plot doesn't really make sense
figure(4)
subplot(3,1,1);
plot(pose_time_stamp,-pose_position_y); hold on
plot(gt_time_stamp,gt_position(:,1));
hdl = legend('$-y$, Vicon system','$+x$, Compass GT');
set(hdl,'Interpreter','latex')
title('Ground Truth Position','Interpreter','latex');
ylabel('$x$ [m]','Interpreter','latex');
subplot(3,1,2);
plot(pose_time_stamp,pose_position_x); hold on;
plot(gt_time_stamp,gt_position(:,2));
hdl = legend('$+x$, Vicon system','$+y$, Compass GT');
set(hdl,'Interpreter','latex')
ylabel('$y$ [m]','Interpreter','latex');
subplot(3,1,3);
plot(pose_time_stamp,pose_position_z); hold on;
plot(gt_time_stamp,gt_position(:,3));
ylabel('$z$ [m]','Interpreter','latex');
% xlim([0, gt_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('$+z$, Vicon system','$+z$, Compass GT');
set(hdl,'Interpreter','latex')

figure(5)
subplot(3,1,1);
plot(pose_time_stamp,pose_position_x); hold on
plot(gt_time_stamp,gt_position(:,1));
hdl = legend('$+x$, Vicon system','$+x$, Compass GT');
set(hdl,'Interpreter','latex')
title('Ground Truth Position','Interpreter','latex');
ylabel('$x$ [m]','Interpreter','latex');
subplot(3,1,2);
plot(pose_time_stamp,pose_position_y); hold on;
plot(gt_time_stamp,gt_position(:,2));
hdl = legend('$+y$, Vicon system','$+y$, Compass GT');
set(hdl,'Interpreter','latex')
ylabel('$y$ [m]','Interpreter','latex');
subplot(3,1,3);
plot(pose_time_stamp,pose_position_z); hold on;
plot(gt_time_stamp,gt_position(:,3));
ylabel('$z$ [m]','Interpreter','latex');
% xlim([0, gt_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('$+z$, Vicon system','$+z$, Compass GT');
set(hdl,'Interpreter','latex')

%% Evaluate Relative Position Error

% plot translation components
figure(6)
subplot(3,1,1);
plot(gt_time_stamp-gt_time_stamp(1),gt_position_x); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_position_x);
plot(VID_time_stamp-gt_time_stamp(1),VID_position_x);
ylabel('$x$ [m]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
subplot(3,1,2);
plot(gt_time_stamp-gt_time_stamp(1),gt_position_y); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_position_y);
plot(VID_time_stamp-gt_time_stamp(1),VID_position_y);
ylabel('$y$ [m]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
subplot(3,1,3);
plot(gt_time_stamp-gt_time_stamp(1),gt_position_z); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_position_z);
plot(VID_time_stamp-gt_time_stamp(1),VID_position_z);
ylabel('$z$ [m]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('ground truth','VI-SLAM','VID-SLAM');
set(hdl,'Interpreter','latex')

% plot velocity vector components
figure(7)
subplot(3,1,1);
plot(gt_time_stamp-gt_time_stamp(1),gt_velocity_x); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_velocity_x);
plot(VID_time_stamp-gt_time_stamp(1),VID_velocity_x);
ylabel('$v_xx$ [m/s]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
subplot(3,1,2);
plot(gt_time_stamp-gt_time_stamp(1),gt_velocity_y); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_velocity_y);
plot(VID_time_stamp-gt_time_stamp(1),VID_velocity_y);
ylabel('$v_y$ [m/s]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
subplot(3,1,3);
plot(gt_time_stamp-gt_time_stamp(1),gt_velocity_z); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_velocity_z);
plot(VID_time_stamp-gt_time_stamp(1),VID_velocity_z);
ylabel('$v_z$ [m/]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
xlabel('time [s]','Interpreter','latex');
hdl = legend('ground truth','VI-SLAM','VID-SLAM');
set(hdl,'Interpreter','latex')

% plot speed
figure(8)
plot(gt_time_stamp-gt_time_stamp(1),gt_speed); hold on;
plot(compass_time_stamp-gt_time_stamp(1),compass_speed);
plot(VID_time_stamp-gt_time_stamp(1),VID_speed);
ylabel('speed [m/s]','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
xlim([0, gt_time_second(end)]);

figure(9)
plot3(gt_position_x,gt_position_y,gt_position_z); hold on;
plot3(compass_position_x,compass_position_y,compass_position_z);
plot3(VID_position_x,VID_position_y,VID_position_z);
xlabel('$x$ [m]','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex');
zlabel('$z$ [m]','Interpreter','latex');
hdl = legend('ground truth','VI-SLAM','VID-SLAM');
set(hdl,'Interpreter','latex')

%% Get Postition Error and RMSE

dim = size(gt_position,2);

[ RMSE_position_compass, error_position_compass ] = getRMSE( compass_position, ...
    compass_time_stamp, gt_position, gt_time_stamp, dim);

[ RMSE_position_VID, error_position_VID ] = getRMSE( VID_position, ...
    VID_time_stamp, gt_position, gt_time_stamp, dim);

disp('RMSE_position_compass, RMSE_Vposition_VID = ')
disp([RMSE_position_compass, RMSE_position_VID]);

% maybe plot abs() of error instead?
figure(10)
subplot(3,1,1);
plot(compass_time_stamp,error_position_compass(:,1)); hold on
plot(VID_time_stamp,error_position_VID(:,1));
ylabel('error, $x$ [m]','Interpreter','latex');
subplot(3,1,2);
plot(compass_time_stamp,error_position_compass(:,2)); hold on
plot(VID_time_stamp,error_position_VID(:,2));
ylabel('error, $y$ [m]','Interpreter','latex');
subplot(3,1,3);
plot(compass_time_stamp,error_position_compass(:,3)); hold on
plot(VID_time_stamp,error_position_VID(:,3));
ylabel('error, $z$ [m]','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
hdl = legend('VI-SLAM','VID-SLAM');
set(hdl,'Interpreter','latex')

%% Get Velocity Error and RMSE

[ RMSE_velocity_compass, error_velocity_compass ] = getRMSE( compass_velocity, ...
    compass_time_stamp, gt_velocity, gt_time_stamp, dim);

[ RMSE_velocity_VID, error_velocity_VID ] = getRMSE( VID_velocity, ...
    VID_time_stamp, gt_velocity, gt_time_stamp, dim);

error_velocity_compass = abs(error_velocity_compass);
error_velocity_VID = abs(error_velocity_VID);

disp('RMSE_velocity_compass, RMSE_velocity_VID = ')
disp([RMSE_velocity_compass, RMSE_velocity_VID]);

% maybe plot abs() of error instead?
figure(11)
subplot(3,1,1);
plot(compass_time_stamp-gt_time_stamp(1),error_velocity_compass(:,1)); hold on
plot(VID_time_stamp-gt_time_stamp(1),error_velocity_VID(:,1));
ylabel('error, $v_xx$ [m/s]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
subplot(3,1,2);
plot(compass_time_stamp-gt_time_stamp(1),error_velocity_compass(:,2)); hold on
plot(VID_time_stamp-gt_time_stamp(1),error_velocity_VID(:,2));
ylabel('error, $v_y$ [m/s]','Interpreter','latex');
xlim([0, gt_time_second(end)]);
subplot(3,1,3);
plot(compass_time_stamp-gt_time_stamp(1),error_velocity_compass(:,3)); hold on
plot(VID_time_stamp-gt_time_stamp(1),error_velocity_VID(:,3));
ylabel('error, $v_z$ [m/s]','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
hdl = legend('VI-SLAM','VID-SLAM');
set(hdl,'Interpreter','latex')
xlim([0, gt_time_second(end)]);

