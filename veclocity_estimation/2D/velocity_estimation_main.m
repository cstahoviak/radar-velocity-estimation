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
filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight'
% filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight_dynamicperson'
% filename = 'rangeRes_0-14_velRes_0-04_velMax_2-26_pkgrp_doppler_zigzagmotion';
% filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_straightmotion'
filetype = '.mat';
mat_file = strcat(path,'/mat_files/',filename,filetype);

load(mat_file);

% bag_file = strcat(path,filename,'.bag');
% bag = rosbag(bag_file);

% list available frames
% bag.AvailableFrames

%% Modify Radar Data

% define start and finish indices to remove unwanted data at start and end
% of test run
start = 3;
finish = 1078;  % flight with only  static targets
% finish = 1070;  % flight with dynamics

% start = 1;
% finish = 1103;

radar_range       = radar_fwd_range(start:finish,:);
radar_intensity   = radar_fwd_intensity(start:finish,:);
radar_doppler     = radar_fwd_doppler(start:finish,:);
radar_time_second = radar_fwd_time_second(start:finish,:);

radar_x = radar_fwd_x(start:finish,:);
radar_y = radar_fwd_y(start:finish,:);

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

% NOTE: Vicon pose message orientation in N.W.U. frame
orientation = [pose_orientation_w, ... 
               pose_orientation_x, ... 
               pose_orientation_y, ...
               pose_orientation_z];

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
max_intensity = max(max(radar_intensity));
min_intensity = min(min(radar_intensity));
% int_range = [max_intenisty; min_intensity];
int_range = [max_intensity; 5];
sigma_vr = 0.044;     % [m/s]
load('radar_angle_bins.mat')

norm_thresh = 1.3;
% norm_thresh = 1e6;

%% Get Vicon Body-Frame Velocities

twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
twist_linear_body = getBodyFrameVelocities( twist_linear, orientation );

% Map from NWU to NED coordinate frame
twist_linear_body(:,2) = -twist_linear_body(:,2);
twist_linear_body(:,3) = -twist_linear_body(:,3);

% twist_linear_body = smoothdata(twist_linear_body,1);

% get euler angles
euler_angles = quat2eul(orientation);

% plot ground truth body-frame velocities
plot(ax_h(1), twist_time_second, twist_linear_body)
hdl = legend(ax_h(1), '$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex','Location','best');
xlim(ax_h(1),[0, radar_time_second(end)]);

% plot euler angles - to visualize pitch and roll
plot(ax_h(2), pose_time_second, rad2deg(euler_angles(:,2:3)));
hdl = legend(ax_h(2), 'pitch, $\phi$', 'roll, $\theta$');
set(hdl,'Interpreter','latex','Location','best');
% ylim(ax_h(2),[-20,20]);
xlim(ax_h(2),[0, radar_time_second(end)]);


%% Implement Estimation Scheme

NScans = size(radar_doppler,1);

% init estimate vectors
vx_hat_mean        = zeros(NScans,1);
vx_hat_max         = zeros(NScans,1);
vhat_bf            = zeros(NScans,2);
vhat_bf_inlier     = zeros(NScans,2);
vhat_MLESAC        = zeros(NScans,2);
vhat_ODR           = zeros(NScans,2);
vhat_ODR_weighted  = zeros(NScans,2);

% init covariance matrices
sigma_odr = NaN*ones(NScans,2);
sigma_odr_weighted = NaN*ones(NScans,2);

% init estimate error vectors
vx_hat_mean_err = zeros(NScans,1);
vx_hat_max_err  = zeros(NScans,1);

std_dev_bf = zeros(NScans,2);

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
    
    % remove NaNs from radar data (and possibly zero-doppler targets)
    idx_pre = prefilter(radar_doppler(i,:), false);
    
    % apply {angle, intensity, range} 'AIR' filtering
    data_AIR = [radar_angle(i,idx_pre)', radar_intensity(i,idx_pre)', ...
        radar_range(i,idx_pre)'];
    [ idx_AIR ] = AIR_filtering( data_AIR, [90, 5, 0.30]);
    
    Ntargets = sum(idx_AIR);    % number of valid targets per scan
%     disp([i, sum(idx_pre), Ntargets]);
    
    if Ntargets > 0
    
        % redefine doppler, angle and intensity vectors to use below
        doppler   = radar_doppler(i,idx_pre);
        angle     = radar_angle(i,idx_pre);
        intensity = radar_intensity(i,idx_pre);
        
        [ Nbins, ~ ] = getNumAngleBins( angle(idx_AIR) );
%         fprintf('Nbins(%d) = %d\n', i, Nbins)

        % get 'brute force' estimate of forward/lateral body-frame vel.
        [ model_bf, vhat_bf_all ] = getBruteForceEstimate( ...
            doppler(idx_AIR), angle(idx_AIR), conditionNum_thres);
        vhat_bf(i,:) = -model_bf';
        std_dev_bf(i,1) = nanstd(vhat_bf_all(1,:));
        std_dev_bf(i,2) = nanstd(vhat_bf_all(2,:));

        % plot these values against the truth to see if they make sense!
        time = ones(1,size(vhat_bf_all,2))*radar_time_second(i);
        scatter(ax_h(4),time,-vhat_bf_all(1,:),sz,colors(1,:),'filled');
        scatter(ax_h(5),time,-vhat_bf_all(2,:),sz,colors(3,:),'filled');
        
%         return;

        % get MLESAC (M-estimator RANSAC) model and inlier set
        [ model_mlesac, inlier_idx ] = MLESAC( doppler(idx_AIR), ...
            angle(idx_AIR), sampleSize, maxDistance, conditionNum_thres );
%         vhat_MLESAC(i,:) = -model_mlesac';
    %     disp([length(radar_doppler(i,idx_pre)), sum(inlier_idx)])
    
        disp([i, sum(idx_pre), Ntargets, sum(inlier_idx)]);
%         fprintf('Ntargets_valid = %d\n', Ntargets)
%         fprintf('Ntargets_inlier = %d\n\n', sum(inlier_idx))
    
        if norm(model_mlesac) < norm_thresh
            vhat_MLESAC(i,:) = -model_mlesac';
        else
            vhat_MLESAC(i,:) = vhat_MLESAC(i-1,:);
        end
    
        % define AIR filtered data sets
        doppler_AIR = doppler(idx_AIR);
        angle_AIR   = angle(idx_AIR);
        intensity_AIR = intensity(idx_AIR);

        % re-run brute-force solver on inlier_idx
        [ model_bf_inlier, vhat_bf_all_inlier ] = getBruteForceEstimate( ...
            doppler_AIR(inlier_idx'), angle_AIR(inlier_idx'), conditionNum_thres);
        vhat_bf_inlier(i,:) = -model_bf_inlier';
        time = ones(1,size(vhat_bf_all_inlier,2))*radar_time_second(i);
        scatter(ax_h(8),time,-vhat_bf_all_inlier(1,:),sz,colors(1,:),'filled');
        scatter(ax_h(9),time,-vhat_bf_all_inlier(2,:),sz,colors(3,:),'filled');

        % Orthogonal Distance Regression (ODR) on inlier set
        Ntargets_inlier = sum(inlier_idx);
        [ numAngleBins, ~ ] = getNumAngleBins( angle_AIR(inlier_idx') );
    %     if Ntargets_inlier > 5 && numAngleBins > 1
        if (Ntargets_inlier > 2) && (numAngleBins > 1)
            delta = normrnd(0,sigma_theta,[Ntargets_inlier,1]);
            
            % get target weights
            weights_const = (1/sigma_vr)*ones(Ntargets_inlier,1);
%             weights_int = odr_getWeights(intensity_AIR(inlier_idx'), sigma_vr,...
%                 int_range);
            int_range2 = [max(intensity_AIR(inlier_idx')); int_range(2)];
            weights_int = odr_getWeights(intensity_AIR(inlier_idx'), sigma_vr,...
                int_range2);
%             weights_int2 = (intensity_AIR(inlier_idx')./sigma_vr)';
            
            % error variance ratio
            d = (sigma_vr/sigma_theta)*ones(Ntargets_inlier,1);
            
            if norm(model_mlesac) < norm_thresh
                % constant weighting scheme
                [ model_odr, ~, cov ] = ODR( angle_AIR(inlier_idx'), ...
                    doppler_AIR(inlier_idx'), d, model_mlesac, delta, weights_const );
                
%               % normalized intenisty weighting scheme 2
                [ model_odr_weighted, ~, cov_weighted ] = ODR( angle_AIR(inlier_idx'), ...
                    doppler_AIR(inlier_idx'), d, model_mlesac, delta, weights_int );
            else
                % constant weighting scheme
                [ model_odr, ~, cov ] = ODR( angle_AIR(inlier_idx'), ...
                    doppler_AIR(inlier_idx'), d, vhat_bf(i,:)', delta, weights_const );
                
%               % normalized intenisty weighting scheme 2
                [ model_odr_weighted, ~, cov_weighted ] = ODR( angle_AIR(inlier_idx'), ...
                    doppler_AIR(inlier_idx'), d, vhat_bf(i,:)', delta, weights_int );
            end
            
            % constant weighting scheme
            if norm(model_odr) < norm_thresh
                vhat_ODR(i,:)  = -model_odr';
            else
                vhat_ODR(i,:)  = vhat_ODR(i-1,:);
            end
            
            % normalized intenisty weighting scheme
            if norm(model_odr_weighted) < norm_thresh
                vhat_ODR_weighted(i,:) = -model_odr_weighted';
            else
                vhat_ODR_weighted(i,:) = vhat_ODR_weighted(i-1,:);
            end
            
            sigma_odr(i,:) = [sqrt(cov(1,1)), sqrt(cov(2,2))];
            sigma_odr_weighted(i,:) = real([sqrt(cov_weighted(1,1)), sqrt(cov_weighted(2,2))]);
            
        else
            vhat_ODR(i,:)          = NaN*ones(1,2);
            vhat_ODR_weighted(i,:) = NaN*ones(1,2);
        end
        
    else
        vhat_bf(i,:)           = NaN*ones(1,2);
        vhat_MLESAC(i,:)       = NaN*ones(1,2);
        vhat_bf_inlier(i,:)    = NaN*ones(1,2);
        vhat_ODR(i,:)          = NaN*ones(1,2);
        vhat_ODR_weighted(i,:) = NaN*ones(1,2);
    end
    
end

%% Compute RMSE for various estimation schemes

% RMSE for the OLD METHOD
RMSE_mean = sqrt(mean(vx_hat_mean_err.^2))
RMSE_max  = sqrt(mean(vx_hat_max_err.^2))

RMSE_bruteforce = getDopplerRMSE( vhat_bf, radar_time_second, ...
    twist_time_second, twist_linear_body)

RMSE_MLESAC = getDopplerRMSE( vhat_MLESAC, radar_time_second, ...
    twist_time_second, twist_linear_body)

RMSE_bruteforcef_inlier = getDopplerRMSE( vhat_bf_inlier, ...
    radar_time_second, twist_time_second, twist_linear_body)

RMSE_ODR = getDopplerRMSE( vhat_ODR, radar_time_second, ...
    twist_time_second, twist_linear_body)

RMSE_ODR_weighted = getDopplerRMSE( vhat_ODR_weighted, radar_time_second, ...
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

cla(ax_h(12));
cla(ax_h(13));

cla(ax_h(14));
cla(ax_h(15));

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
hdl = legend(ax_h(7),'Vicon system','ODR');
set(hdl,'Interpreter','latex','Location','northwest')

% inlier set - brute force method
plot(ax_h(8),twist_time_second,twist_linear_body(:,1), ...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(8),radar_time_second,vhat_bf_inlier(:,1),'k','LineWidth',1);
plot(ax_h(9),twist_time_second,twist_linear_body(:,2), ...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(9),radar_time_second,vhat_bf_inlier(:,2),'k','LineWidth',1);
ylim(ax_h(8),[-2,2]); ylim(ax_h(9),[-1,1]);
xlim(ax_h(8), [0, radar_time_second(end)]);
xlim(ax_h(9), [0, radar_time_second(end)]);

% plot weighted ODR estimate
plot(ax_h(10),twist_time_second,twist_linear_body(:,1), ...
    'color',colors(2,:),'LineWidth',2);
% plot(ax_h(10),twist_time_second,twist_linear_x,'LineWidth',2);
plot(ax_h(10),radar_time_second,vhat_ODR_weighted(:,1),'k','LineWidth',1);
plot(ax_h(11),twist_time_second,twist_linear_body(:,2), ...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(11),radar_time_second,vhat_ODR_weighted(:,2),'k','LineWidth',1);
ylim(ax_h(10),[-1,1.5]); ylim(ax_h(11),[-1,1]);
xlim(ax_h(10), [0, radar_time_second(end)]);
xlim(ax_h(11), [0, radar_time_second(end)]);
hdl = legend(ax_h(11),'Vicon system','weighted ODR');
set(hdl,'Interpreter','latex','Location','northwest')

% plot Weighted ODR + Brute-Force estimates
plot(ax_h(12),twist_time_second,twist_linear_body(:,1),...
    'color',colors(2,:),'LineWidth',2);
% plot(ax_h(10),twist_time_second,twist_linear_x,'LineWidth',2);
plot(ax_h(12),radar_time_second,vhat_ODR_weighted(:,1),'k','LineWidth',1);
plot(ax_h(12),radar_time_second,vhat_bf(:,1),'Color',colors(1,:),'LineStyle',':');
plot(ax_h(13),twist_time_second,twist_linear_body(:,2),...
    'color',colors(2,:),'LineWidth',2);
plot(ax_h(13),radar_time_second,vhat_ODR_weighted(:,2),'k','LineWidth',1);
plot(ax_h(13),radar_time_second,vhat_bf(:,2),'Color',colors(1,:),'LineStyle',':');
ylim(ax_h(12),[-1,1.5]); ylim(ax_h(13),[-1,1]);
xlim(ax_h(12), [0, radar_time_second(end)]);
xlim(ax_h(13), [0, radar_time_second(end)]);
% hdl = legend(ax_h(13),'Vicon system','ODR','brute-force');
% set(hdl,'Interpreter','latex','Location','northwest')

% add vertical line markers to above plot - indicates dynamics in scene
loc = [12.3, 18.2, 55.4, 60.2, 81.8, 87.1];
for i=1:length(loc)
    plot(ax_h(12),[loc(i), loc(i)], [-1, 2],'r--')
    plot(ax_h(13),[loc(i), loc(i)], [-1, 2],'r--')
end
hdl = legend(ax_h(13),'Vicon system','weighted ODR','brute-force');
set(hdl,'Interpreter','latex','Location','northwest')

K = 10;
% plot weighted ODR + covariance bounds
plot(ax_h(14),radar_time_second,vhat_ODR_weighted(:,1),'k','LineWidth',1);
plot(ax_h(14),radar_time_second,vhat_ODR_weighted(:,1) + ...
    K*sigma_odr_weighted(:,1),'r--');
plot(ax_h(14),radar_time_second,vhat_ODR_weighted(:,1) - ...
    K*sigma_odr_weighted(:,1),'r--');
plot(ax_h(15),radar_time_second,vhat_ODR_weighted(:,2),'k','LineWidth',1);
plot(ax_h(15),radar_time_second,vhat_ODR_weighted(:,2) + ...
    K*sigma_odr_weighted(:,1),'r--');
plot(ax_h(15),radar_time_second,vhat_ODR_weighted(:,2) - ...
    K*sigma_odr_weighted(:,1),'r--');
ylim(ax_h(14),[-1,1.5]); ylim(ax_h(15),[-1,1]);
xlim(ax_h(14), [0, radar_time_second(end)]);
xlim(ax_h(15), [0, radar_time_second(end)]);
hdl = legend(ax_h(14),'weighted ODR','2$\sigma$ envelope');
set(hdl,'Interpreter','latex','Location','northwest')

figure(20)
plot(radar_time_second,vecnorm(vhat_MLESAC')); hold on;
% plot(radar_time_second,vecnorm(vhat_bf'), '--');

figure(21)
plot(radar_time_second,vecnorm(vhat_ODR_weighted')); hold on;

figure(22)
plot(radar_time_second,std_dev_bf(:,1)); hold on;
plot(radar_time_second,std_dev_bf(:,2));
% plot(radar_time_second,vecnorm(vhat_bf'), '--');

figure(23)
plot(radar_time_second,vhat_ODR_weighted(:,1),'k','LineWidth',1);

figure(24)
plot(radar_time_second,sigma_odr_weighted(:,1)); hold on
plot(radar_time_second,sigma_odr_weighted(:,2))
xlabel('time [s]','Interpreter','latex');
ylabel('standard deviation, $\sigma$ [m/s]','Interpreter','latex')
hdl = legend('$\sigma_{v_x}$','$\sigma_{v_y}$');
set(hdl,'Interpreter','latex','Location','northwest')
xlim([0, radar_time_second(end)]);