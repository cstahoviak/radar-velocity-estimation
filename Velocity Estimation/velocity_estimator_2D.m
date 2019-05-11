%% Header

%%% Filename:   velocity_estimator_2D.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact

% Task: To estimate the forward and lateral body frame velocities of the sensor
% platfrom given input data from a single radar (/mmWaveDataHdl/RScan topic).
% The velocity estimation scheme takes the following approach:
% 
% 1. Near-field targets are removed from the target list. Many of these targets
% are artifacts of antenna interference at the senor origin, and are not
% representative of real targets in the scene. These near-field targets also exist
% in the zero-doppler bin and thus would currupt the quality of the velocity
% estimate.
% 2. A RANSAC (or MLESAC) outlier rejection method is used to filter targets that
% can be attributed to noise or dynamic targets in the environment. RANSAC
% generates an inlier set of targets and a first-pass velocity estimate derived
% from the inlier set.
% 3. Orthogonal Distance Regression (ODR) is seeded with the RANSAC velocity
% estimate and generates a final estimate of the body frame linear velocity
% components.

%% Load Data and ROS bag

path     = '/home/carl/Data/subT/Fleming/multiradar_2019-03-31_velocity_estimation/';
device   = '1642/';
filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight'
% filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_flight_dynamicperson'
% filename = 'rangeRes_0-14_velRes_0-04_velMax_2-26_pkgrp_doppler_zigzagmotion';
% filename = 'rangeRes_0-06_velRes_0-04_velMax_1-28_pkgrp_doppler_straightmotion'
filetype = '.mat';
mat_file = strcat(path,'/mat_files/',filename,filetype);

load(mat_file);

%% Modify Radar Data

% define start and finish indices to remove unwanted data at start and end
% of test run
start = 3;
% finish = 1078;  % flight with only  static targets
finish = 1070;  % flight with dynamics

start = 1;
finish = 1103;

radar_range       = radar_fwd_range(start:finish,:);
radar_intensity   = radar_fwd_intensity(start:finish,:);
radar_doppler     = radar_fwd_doppler(start:finish,:);
radar_time_second = radar_fwd_time_second(start:finish,:);

radar_x = radar_fwd_x(start:finish,:);
radar_y = radar_fwd_y(start:finish,:);

% undo RVIZ plotting defaults
radar_y = -radar_y;

% calculate radar angle
radar_zimuth = atan(radar_y./radar_x);   % [rad]

%% Create Figures

[ fig_h, ax_h ] = createVelocityEstimationPlots_2D();

% load colors for plotting
load('colors.mat');
sz = 4;     % scatter plot marker size

%% Define Constants

% set filtering thresholds
range_thres     = 0.5;      % [m]
azimuth_thres     = 30;       % [deg]
intensity_thres = 23;       % [dB]
thresholds_AIR  = [azimuth_thres; intensity_thres; range_thres];

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

norm_thresh = 1.4;
norm_thresh = 1e6;

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
vhat_MLESAC        = NaN*ones(NScans,2);
vhat_ODR_weighted  = NaN*ones(NScans,2);
vhat               = NaN*ones(NScans,2);

% init covariance matrices
sigma_odr = NaN*ones(NScans,2);
sigma_odr_weighted = NaN*ones(NScans,2);

for i=1:NScans
    
    %%% ORTHOGONAL DISTANCE REGRESSION ON MLESAC INLIER SET %%%%%%%%%%%%%%%
    
    Ntargets = sum( ~isnan(radar_doppler) ); 
    
    % remove NaNs and apply {angle, intensity, range} 'AIR' filtering
    data_AIR = [radar_angle(i,:)', radar_intensity(i,:)', radar_range(i,:)'];
    [ idx_AIR ] = AIR_filtering( data_AIR, [90, 5, 0.30]);
    
    % redefine doppler, angle and intensity vectors to use below
    doppler   = radar_doppler(i,idx_AIR);
    azimuth   = radar_angle(i,idx_AIR);
    intensity = radar_intensity(i,idx_AIR);
    
    % number of valid targets per scan
    Ntargets_valid = sum(idx_AIR);
    % number of azimuth bins that valid targets fall into
    [ Nbins, ~ ] = getNumAngleBins( azimuth );
    
    if Ntargets_valid > 1 && Nbins > 1
    
        % get MLESAC (M-estimator RANSAC) model and inlier set
        [ model_mlesac, inlier_idx ] = MLESAC( doppler, ...
            azimuuth, sampleSize, maxDistance, conditionNum_thres );
%         vhat_MLESAC(i,:) = -model_mlesac';
        Ntargets_inlier = sum(inlier_idx);
    
        if norm(model_mlesac) < norm_thresh
            vhat_MLESAC(i,:) = -model_mlesac';
        else
            vhat_MLESAC(i,:) = vhat_MLESAC(i-1,:);
        end
        
        disp([i, Ntargets, Ntargets_valid, Ntargets_inlier]);

        % Orthogonal Distance Regression (ODR) on inlier set
        int_range_scaled = [max(intensity_AIR(inlier_idx')); int_range(2)];
        weights_int = odr_getWeights(intensity_AIR(inlier_idx'), sigma_vr,...
                        int_range_scaled);
                    
        %%% Stopped Here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
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
            sigma_odr_weighted(i,:) = [sqrt(cov_weighted(1,1)), sqrt(cov_weighted(2,2))];
            
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
