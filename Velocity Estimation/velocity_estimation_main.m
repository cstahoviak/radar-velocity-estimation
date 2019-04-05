%% Header

%%% Filename:   velocity_estimation_main.m
%%% Author:     Carl Stahoviak
%%% Created:    03/04/2019  

clear;
clc;
close all;

format compact

%%% TODO Items
    % 1. Use fmincon (or similar) to search over the set of possible
    % maxDistance values that will minimize the RMSE value of the estimates
    % provided by MLESAC
    
%%% General Notes:
    % 1. Need to perform a timestamp alignment between the Vicon system and
    % the radar data... Chris mentioned this, not sure how to do it though.
    
    % 2. There is a 'hole' in the MLESAC estimate where the quad is
    % momentarily stationary (zero doppler). By throwing away ALL zero
    % doppler points (and not just those that are spurious data points,
    % e.g. constant intensity, near-range sensor interference), we lose the
    % ability to provide a velocity estimate when the quad is stationary.
    % This is a problem. - DONE

%% Load Data and ROS bag

% path     = '/home/carl/Data/subT/Fleming/single_radar_velocity_estimate_2018_11_18/';
% device   = '1642/';
% filename = 'zigzag_best_velocity_res_pkgrp_doppler';
% filetype = '.mat';
% mat_file = strcat(path,device,'/mat_files/',filename,filetype);

path     = '/home/carl/Data/subT/Fleming/multiradar_2019-03-31_velocity_estimation/';
device   = '1642/';
filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight';
filetype = '.mat';
mat_file = strcat(path,'/mat_files/',filename,filetype);

load(mat_file);

% bag_file = strcat(path,filename,'.bag');
% bag = rosbag(bag_file);

% list available frames
% bag.AvailableFrames

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

[ fig_h, ax_h ] = createVelocityEstimationPlots();

% load colors for plotting
load('colors.mat');
sz = 4;     % scatter plot marker size

%% Topic Information

radar_topic       = '/mmWaveDataHdl/RScan';
vrpn_twist_topic  = '/vrpn_client_node/SubT/twist';
vrpn_pose_topic   = '/vrpn_client_node/SubT/pose';
tf_topic          = '/tf';

% create concatenated position and orientation matrices
position = [pose_position_x, ...
            pose_position_y, ...
            pose_position_z];
        
orientation = [pose_orientation_x, ... 
               pose_orientation_y, ... 
               pose_orientation_z, ...
               pose_orientation_w];

%% Define Constants

% set filtering thresholds
range_thres     = 0.5;      % [m]
angle_thres     = 30;       % [deg]
intensity_thres = 23;       % [dB]
thresholds_AIR  = [angle_thres; intensity_thres; range_thres];

% remove zero doppler velocity points?
filter_nonZero = true;

% define MLESAC parameters
sampleSize         = 2;     % problem uniquely-determined for 2 targets
maxDistance        = 0.1;   % only roughly tuned at this point
conditionNum_thres = 100;   % NOTE: also used to the brute-froce estimate

% define ODR parameters
sigma_vr = 0.044;     % [m/s]
load('radar_angle_bins.mat')

%% Get Vicon Body-Frame Velocities

twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
twist_linear_body = getBodyFrameVelocities( twist_linear, orientation );
twist_linear_body(:,1) = -twist_linear_body(:,1);
twist_linear_body(:,2) = -twist_linear_body(:,2);

twist_linear_body = smoothdata(twist_linear_body,1);

% get euler angles
euler_angles = quat2eul(orientation);

% undo RVIZ plotting defaults
twist_linear_body(:,2) = -twist_linear_body(:,2);

% plot ground truth body-frame velocities
plot(ax_h(1), twist_time_second, twist_linear_body)
hdl = legend(ax_h(1), '$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex','Location','best');
xlim([0, radar_time_second(end)]);

% plot euler angles - to visualize pitch and roll
plot(ax_h(2), pose_time_second, rad2deg(euler_angles(:,2:3)));
hdl = legend(ax_h(2), 'pitch, $\phi$', 'roll, $\theta$');
set(hdl,'Interpreter','latex','Location','best');
% ylim(ax_h(2),[-20,20]);
xlim([0, radar_time_second(end)]);


%% Implement Estimation Scheme

NScans = size(radar_doppler,1);

% init estimate vectors
vx_hat_mean    = zeros(NScans,1);
vx_hat_max     = zeros(NScans,1);
vhat_bf        = zeros(NScans,2);
vhat_bf_inlier = zeros(NScans,2);
vhat_MLESAC    = zeros(NScans,2);
vhat_ODR       = zeros(NScans,2);

% init estimate error vectors
vx_hat_mean_err = zeros(NScans,1);
vx_hat_max_err  = zeros(NScans,1);

for i=1:NScans
    
    %%% OLD METHOD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % remove NaN and (possibly) zero-doppler values
    % NOTE: zero-doppler values have been shown to be artifacts of constant
    % intensity targets that are not representative of physical targets
    idx_pre = prefilter(radar_doppler(i,:), true);
    
    % concatenate radar data (into column vectors)
    radar_data = [radar_range(i,:)', radar_angle(i,:)', ...
        radar_doppler(i,:)', radar_intensity(i,:)'];
    
    % apply filtering and scaling (mathematically incorrect) - OLD METHOD
    [ doppler_scaled, ~ ] = applyFilteringandScaling( ...
        radar_data, thresholds_AIR, idx_pre);
    
    % get forward velocity estimate - OLD METHOD
    [ vx_hat_mean(i,1), vx_hat_max(i,1), ...
        vx_hat_mean_err(i,1), vx_hat_max_err(i,1) ] = ...
        getForwardVelocityEstimate( doppler_scaled, radar_time_second(i), ...
        twist_time_second, -twist_linear_body );
    
    %%% NEW METHOD %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % filter close range points - remove constant int. targets
    idx_pre = prefilter(radar_doppler(i,:), false);
    
    % apply {angle, intensity, range} 'AIR' filtering
    data_AIR = [radar_angle(i,idx_pre)', radar_intensity(i,idx_pre)', ...
        radar_range(i,idx_pre)'];
    [ idx_AIR ] = AIR_filtering( data_AIR, [90, 5, 0.30]);
    
    Ntargets = sum(idx_AIR);    % number of valid targets per scan
    disp([i, sum(idx_pre), Ntargets]);
    
    if Ntargets > 0
    
        % redefine doppler, angle and intensity vectors to use below
        doppler   = radar_doppler(i,idx_pre);
        angle     = radar_angle(i,idx_pre);
        intensity = radar_intensity(i,idx_pre);

        % get 'brute force' estimate of forward/lateral body-frame vel.
        [ model_bf, vhat_bf_all ] = getBruteForceEstimate( ...
            doppler(idx_AIR), angle(idx_AIR), conditionNum_thres);
        vhat_bf(i,:) = -model_bf';

        % plot these values against the truth to see if they make sense!
        time = ones(1,size(vhat_bf_all,2))*radar_time_second(i);
        scatter(ax_h(4),time,vhat_bf_all(1,:),sz,colors(1,:),'filled');
        scatter(ax_h(5),time,vhat_bf_all(2,:),sz,colors(3,:),'filled');

        % get MLESAC (M-estimator RANSAC) model and inlier set
        [ model_mlesac, inlier_idx ] = MLESAC( doppler(idx_AIR), ...
            angle(idx_AIR), sampleSize, maxDistance, conditionNum_thres );
        vhat_MLESAC(i,:) = -model_mlesac';
    %     disp([length(radar_doppler(i,idx_pre)), sum(inlier_idx)])

        % re-run brute-force solver on inlier_idx
        [ model_bf_inlier, vhat_bf_all_inlier ] = getBruteForceEstimate( ...
            doppler(inlier_idx'), angle(inlier_idx'), conditionNum_thres);
        vhat_bf_inlier(i,:) = -model_bf_inlier';
        time = ones(1,size(vhat_bf_all_inlier,2))*radar_time_second(i);
        scatter(ax_h(8),time,vhat_bf_all_inlier(1,:),sz,colors(1,:),'filled');
        scatter(ax_h(9),time,vhat_bf_all_inlier(2,:),sz,colors(3,:),'filled');

        % Orthogonal Distance Regression (ODR) on inlier set
        Ntargets_inlier = sum(inlier_idx);
        [ numAngleBins, ~ ] = getNumAngleBins( angle(inlier_idx) );
    %     if Ntargets_inlier > 5 && numAngleBins > 1
        if numAngleBins > 1
            weights = (1/sigma_vr)*ones(Ntargets_inlier,1);
            delta = normrnd(0,sigma_theta,[Ntargets_inlier,1]);
            d = (sigma_vr/sigma_theta)*ones(Ntargets_inlier,1);    % error variance ratio
            [ model_odr, ~ ] = ODR( angle(inlier_idx), doppler(inlier_idx), ...
                d, model_mlesac, delta, weights );
            vhat_ODR(i,:) = -model_odr';
        else
            vhat_ODR(i,:) = NaN*ones(1,2);
        end
        
    else
        vhat_bf(i,:)        = NaN*ones(1,2);
        vhat_MLESAC(i,:)    = NaN*ones(1,2);
        vhat_bf_inlier(i,:) = NaN*ones(1,2);
        vhat_ODR(i,:)       = NaN*ones(1,2);
    end
    
end

%% Compute RMSE for various estimation schemes

% RMSE for the OLD METHOD
RMSE_mean = sqrt(mean(vx_hat_mean_err.^2))
RMSE_max  = sqrt(mean(vx_hat_max_err.^2))

RMSE_bf = getDopplerRMSE( vhat_bf, radar_time_second, ...
    twist_time_second, twist_linear_body)

RMSE_MLESAC = getDopplerRMSE( vhat_MLESAC, radar_time_second, ...
    twist_time_second, twist_linear_body)

RMSE_bf_inlier = getDopplerRMSE( vhat_bf_inlier, radar_time_second, ...
    twist_time_second, twist_linear_body)

RMSE_ODR = getDopplerRMSE( vhat_ODR, radar_time_second, ...
    twist_time_second, twist_linear_body)

% vhat_old_mean = [-vx_hat_mean, zeros(NScans,1)];
% RMSE_mean2 = getDopplerRMSE( vhat_old_mean, radar_time_second, ...
%     twist_time_second, twist_linear_body)

%% Plot Data

cla(ax_h(3));

cla(ax_h(6));
cla(ax_h(7));

cla(ax_h(10));
cla(ax_h(11));

% twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
% twist_linear_body = getBodyFrameVelocities( twist_linear, orientation );

% forward velocity (v_x) estimate vs. truth - OLD METHOD
plot(ax_h(3), twist_time_second,twist_linear_body(:,1),'color',colors(2,:));
plot(ax_h(3), radar_time_second,vx_hat_mean,'color',colors(1,:));
plot(ax_h(3), radar_time_second,vx_hat_max,'color',colors(5,:));
hdl = legend(ax_h(3), '$v_x$ Vicon','mean estimate','max estimate');
set(hdl,'Interpreter','latex','Location','best');
xlim(ax_h(3), [0, radar_time_second(end)]);

% add estimate + truth data to plot 3 - brute force method
plot(ax_h(4),twist_time_second,twist_linear_body(:,1),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(4),radar_time_second,vhat_bf(:,1),'k','LineWidth',1);
plot(ax_h(5),twist_time_second,twist_linear_body(:,2),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(5),radar_time_second,vhat_bf(:,2),'k','LineWidth',1);
ylim(ax_h(4),[-2,2]); ylim(ax_h(5),[-1,1]);
xlim(ax_h(4), [0, radar_time_second(end)]);
xlim(ax_h(5), [0, radar_time_second(end)]);

% plot MLESAC estimate
plot(ax_h(6),twist_time_second,twist_linear_body(:,1),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(6),radar_time_second,vhat_MLESAC(:,1),'k','LineWidth',1);
plot(ax_h(7),twist_time_second,twist_linear_body(:,2),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(7),radar_time_second,vhat_MLESAC(:,2),'k','LineWidth',1);
% ylim(ax_h(6),[-1.5,1.5]); ylim(ax_h(7),[-1,1]);
xlim(ax_h(6), [0, radar_time_second(end)]);
xlim(ax_h(7), [0, radar_time_second(end)]);

% inlier set - brute force method
plot(ax_h(8),twist_time_second,twist_linear_body(:,1),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(8),radar_time_second,vhat_bf_inlier(:,1),'k','LineWidth',1);
plot(ax_h(9),twist_time_second,twist_linear_body(:,2),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(9),radar_time_second,vhat_bf_inlier(:,2),'k','LineWidth',1);
ylim(ax_h(8),[-2,2]); ylim(ax_h(9),[-1,1]);
xlim(ax_h(8), [0, radar_time_second(end)]);
xlim(ax_h(9), [0, radar_time_second(end)]);

% plot ODR estimate
plot(ax_h(10),twist_time_second,twist_linear_body(:,1),...
    'color',colors(2,:),'LineWidth',2);
% plot(ax_h(10),twist_time_second,twist_linear_x,'LineWidth',2);
plot(ax_h(10),radar_time_second,vhat_ODR(:,1),'k','LineWidth',1);
plot(ax_h(11),twist_time_second,twist_linear_body(:,2),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(11),radar_time_second,vhat_ODR(:,2),'k','LineWidth',1);
% ylim(ax_h(10),[-1.5,1.5]); ylim(ax_h(11),[-0.5,0.5]);
xlim(ax_h(10), [0, radar_time_second(end)]);
xlim(ax_h(11), [0, radar_time_second(end)]);

