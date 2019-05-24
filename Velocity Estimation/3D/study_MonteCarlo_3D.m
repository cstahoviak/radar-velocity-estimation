%% Header

%%% Filename:   study_MonteCarlo_3D.m
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

path     = '/home/carl/Data/subT/Fleming/fleming_radar_2019-05-17/';
filename = 'cfar-1000_10Hz_run0';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

%% Modify Radar Data

% define start and finish indices to remove unwanted data at start and end of test run
% start = 3;
% finish = 1078;  % flight with only  static targets
% finish = 1070;  % flight with dynamics

% undo RVIZ plotting defaults
radar_y = -radar_y;
radar_z = -radar_z;

% calculate radar angle
radar_azimuth = atan(radar_y./radar_x);         % [rad]
radar_elevation = asin(radar_z./radar_range);   % [rad]

%% Create Figures

[ fig_h, ax_h ] = createVelocityEstimationPlots_3D();

% not used in this version
% close(fig_h(1));
% close(fig_h(2));
close(fig_h(3));
close(fig_h(5));
close(fig_h(7));

% load colors for plotting
load('colors.mat');
sz = 4;     % scatter plot marker size

%% Define Constants

% set filtering thresholds
range_thres     = 0.3;      % [m]
azimuth_thres   = 85;       % [deg]
elevation_thres = 85;       % [deg]
intensity_thres = 1;        % [dB]
thresholds_AIRE = [azimuth_thres; intensity_thres; range_thres; elevation_thres];

% define MLESAC parameters
sampleSize         = 3;     % problem uniquely-determined for 2 targets
maxDistance        = 0.15;   % only roughly tuned at this point

% define ODR parameters
max_intensity = max(max(radar_intensity));
min_intensity = min(min(radar_intensity));
int_range = [max_intensity; min_intensity];
sigma_vr = 0.044;           % [m/s]
load('1642_azimuth_bins.mat')
sigma_phi = sigma_theta;    % for now - need to conduct elev. bin study.

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

% norm_thresh = 1.3;
norm_thresh = inf;

%% Get Vicon Body-Frame Velocities

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

% twist_linear_body = smoothdata(twist_linear_body,1);

% get euler angles
euler_angles = quat2eul(orientation);

% plot ground truth body-frame velocities
plot(ax_h(1), twist_time_second, twist_linear_body)
hdl = legend(ax_h(1), '$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex','Location','best');
xlim(ax_h(1),[0, twist_time_second(end)]);

% plot euler angles - to visualize pitch and roll
plot(ax_h(2), pose_time_second, rad2deg(euler_angles(:,2:3)));
hdl = legend(ax_h(2), 'pitch, $\phi$', 'roll, $\theta$');
set(hdl,'Interpreter','latex','Location','best');
% ylim(ax_h(2),[-20,20]);
xlim(ax_h(2),[0, pose_time_second(end)]);

figure(3)
plot(twist_time_second, vecnorm(twist_linear_body,2,2)); hold on;
plot(twist_time_second(1:end-1), abs(diff(vecnorm(twist_linear_body,2,2))))
ylabel('body-frame velocity vector norm','Interpreter','latex')
xlabel('time [s]','Interpreter','latex');
xlim([0, twist_time_second(end)]);

%% Implement Estimation Scheme

p = 3;  % dimension of velocity estimate vector
NScans = size(radar_doppler,1);

thresholds = [0.0001, 0.0002, 0.0005, 0.001, 0.005, 0.009]';
thresholds = repelem(thresholds,5);
% thresholds = sort(thresholds,'descend');

mc_study = NaN*ones(size(thresholds,1),8);
MCiter = size(thresholds,1);

% return;

for m=1:MCiter
    
    fprintf('Monte Carlo interation: %d\n', m);

    % init estimate vectors
    vhat_MLESAC        = NaN*ones(NScans,p);
    vhat_ODR_weighted  = NaN*ones(NScans,p);

    % init covariance matrices
    sigma_odr_weighted = NaN*ones(NScans,p);

    % data to save to csv file
    data = NaN*ones(NScans,10);  % [stamp, vx, vy, sigsq_x, sigsq_y, sigsq_z, sig_xy, sig_xz, sig_yz]
    data(:,1) = radar_time_stamp;

    targets = NaN*ones(NScans,3);   % [Ntargets, Ntargets_valid, Ntargets_inlier]
    bins = NaN*ones(NScans,2);      % [Nbins_valid, Nbins_inlier]
    odr_iter = NaN*ones(NScans,1);  % number of iteratiosn required for ODR convergence

    for i=1:NScans

        %%% ORTHOGONAL DISTANCE REGRESSION ON MLESAC INLIER SET %%%%%%%%%%%%%%%

        Ntargets = sum( ~isnan(radar_doppler(i,:)) );
        targets(i,1) = Ntargets;

        % remove NaNs and apply {angle, intensity, range} 'AIR' filtering
        data_AIRE = [radar_azimuth(i,:)', radar_intensity(i,:)', ...
                     radar_range(i,:)', radar_elevation(i,:)'];
        [ idx_AIRE ] = AIRE_filtering( data_AIRE, thresholds_AIRE);

        % redefine doppler, angle and intensity vectors to use below
        doppler   = radar_doppler(i,idx_AIRE);
        azimuth   = radar_azimuth(i,idx_AIRE);
        elevation = radar_elevation(i,idx_AIRE);
        intensity = radar_intensity(i,idx_AIRE);

        % number of valid targets per scan
        Ntargets_valid = sum(idx_AIRE);
        targets(i,2) = Ntargets_valid;

        if Ntargets_valid > 2
            % number of azimuth bins that valid targets fall into
            [ Nbins_valid, ~ ] = getNumAngleBins( azimuth );
            bins(i,1) = Nbins_valid;

            % NOTE: a valid velocity estimate can only be derived for 2 or more
            % targets located at distinct azimuth bins
            if Ntargets_valid > 1 %&& Nbins_valid > 1

                % get MLESAC (Maximum Likelihood RANSAC) model and inlier set
                [ model_mlesac, inlier_idx ] = MLESAC_3D( doppler, azimuth, ...
                    elevation, sampleSize, maxDistance );
                Ntargets_inlier = sum(inlier_idx);
                targets(i,3) = Ntargets_inlier;

                % reject 'bad' estimates
                if norm(model_mlesac) < norm_thresh
                    vhat_MLESAC(i,:) = -model_mlesac';
                elseif i > 1
                    vhat_MLESAC(i,:) = vhat_MLESAC(i-1,:);
                else
                    % do nothing
                end

                disp([i, Ntargets, Ntargets_valid, Ntargets_inlier]);

                [ Nbins_inlier, ~ ] = getNumAngleBins( azimuth(inlier_idx') );
                bins(i,2) = Nbins_inlier;

                if (Ntargets_inlier > 2) % && (Nbins_inlier > 1)
                    % get target weights
                    int_range_scaled = [max(intensity(inlier_idx')); int_range(2)];
                    weights = odr_getWeights(intensity(inlier_idx'), sigma_vr, int_range_scaled);

                    delta = normrnd(0,sigma_theta,[2*Ntargets_inlier,1]);
                    [ model_odr_weighted, ~, cov_weighted, odr_iter(i) ] = ODR_3D( doppler(inlier_idx'), ...
                        azimuth(inlier_idx'), elevation(inlier_idx'), d, ...
                        model_mlesac, delta, weights, thresholds(m) );

                    % get standard deviations for vx and vy
                    sigma_odr_weighted(i,:) = sqrt(diag(cov_weighted));

                    % reject 'bad' estimates
                    if norm(model_odr_weighted) < norm_thresh
                        vhat_ODR_weighted(i,:) = -model_odr_weighted';
                    elseif i > 1
                        vhat_ODR_weighted(i,:) = vhat_ODR_weighted(i-1,:);
                        sigma_odr_weighted(i,:) = sigma_odr_weighted(i-1,:);
                    else
                        % do nothing
                    end

                    data(i,2:end) = [vhat_ODR_weighted(i,:), diag(cov_weighted)', ...
                                     cov_weighted(1,2), cov_weighted(1,3), cov_weighted(2,3)];

                else
                    vhat_ODR_weighted(i,:) = NaN*ones(1,p);
                end

            else
                vhat_MLESAC(i,:)       = NaN*ones(1,p);
                vhat_ODR_weighted(i,:) = NaN*ones(1,p);
            end

        else
            vhat_MLESAC(i,:)       = NaN*ones(1,p);
            vhat_ODR_weighted(i,:) = NaN*ones(1,p);
        end
    end
    
    % get RMSE values of current run
    [RMSE_mlesac, error_mlesac] = getDopplerRMSE( vhat_MLESAC, radar_time_second, ...
        twist_time_second, twist_linear_body, p);

    [RMSE_odr_weighted, error_odr] = getDopplerRMSE( vhat_ODR_weighted, ...
        radar_time_second, twist_time_second, twist_linear_body, p);
    
    mc_study(m,:) = [thresholds(m), RMSE_mlesac', RMSE_odr_weighted', ...
        mean(odr_iter-1)];
    
end

%% Plot Results

figure(10)
plot(mc_study(:,1), mc_study(:,2),'-o'); hold on
plot(mc_study(:,1), mc_study(:,3),'-o');
plot(mc_study(:,1), mc_study(:,4),'-o');
xlabel('convergence threshold','Interpreter','latex')
ylabel('RMSE [m/s]','Interpreter','latex')
title({'Monte Carlo Study - MLESAC','cfar-800\_10Hz\_run0'},'Interpreter','latex')
hdl = legend('$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex')

figure(11)
plot(mc_study(:,1), mc_study(:,5),'-o'); hold on
plot(mc_study(:,1), mc_study(:,6),'-o');
plot(mc_study(:,1), mc_study(:,7),'-o');
xlabel('convergence threshold','Interpreter','latex')
ylabel('RMSE [m/s]','Interpreter','latex')
title({'Monte Carlo Study - weighted ODR','cfar-800\_10Hz\_run0'},'Interpreter','latex')
hdl = legend('$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex')

figure(12)
plot(mc_study(:,1), mc_study(:,8),'-o')
xlabel('convergence threshold','Interpreter','latex')
ylabel('ODR iterations','Interpreter','latex')
title({'Monte Carlo Study - ODR Iterations','cfar-800\_10Hz\_run0'},'Interpreter','latex')
hdl = legend('ODR iterations');
set(hdl,'Interpreter','latex')

RMSE_avg = zeros(6,7);
RMSE_std = zeros(6,7);
odr_iter_avg = zeros(6,2);

RMSE_avg(:,1) = [0.0001, 0.0002, 0.0005, 0.001, 0.005, 0.009]';
odr_iter_avg(:,1) = [0.0001, 0.0002, 0.0005, 0.001, 0.005, 0.009]';

k = 1;
for i=1:5:size(thresholds,1)
    disp(i)
    RMSE_avg(k,2) = mean(mc_study(i:i+4,2));
    RMSE_avg(k,3) = mean(mc_study(i:i+4,3));
    RMSE_avg(k,4) = mean(mc_study(i:i+4,4));
    RMSE_avg(k,5) = mean(mc_study(i:i+4,5));
    RMSE_avg(k,6) = mean(mc_study(i:i+4,6));
    RMSE_avg(k,7) = mean(mc_study(i:i+4,7));
    
    RMSE_std(k,2) = std(mc_study(i:i+4,2));
    RMSE_std(k,3) = std(mc_study(i:i+4,3));
    RMSE_std(k,4) = std(mc_study(i:i+4,4));
    RMSE_std(k,5) = std(mc_study(i:i+4,5));
    RMSE_std(k,6) = std(mc_study(i:i+4,6));
    RMSE_std(k,7) = std(mc_study(i:i+4,7));
    
    odr_iter_avg(k,2) = mean(mc_study(i:i+4,8));

    k = k+1;
end

figure(13)
errorbar(RMSE_avg(:,1),RMSE_avg(:,2),RMSE_std(:,2)); hold on
errorbar(RMSE_avg(:,1),RMSE_avg(:,3),RMSE_std(:,3));
errorbar(RMSE_avg(:,1),RMSE_avg(:,4),RMSE_std(:,4));
xlabel('convergence threshold','Interpreter','latex')
ylabel('RMSE [m/s]','Interpreter','latex')
title({'Monte Carlo Study - MLESAC','cfar-800\_10Hz\_run0'},'Interpreter','latex')
hdl = legend('$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex')

figure(14)
errorbar(RMSE_avg(:,1),RMSE_avg(:,5),RMSE_std(:,5)); hold on
errorbar(RMSE_avg(:,1),RMSE_avg(:,6),RMSE_std(:,6));
errorbar(RMSE_avg(:,1),RMSE_avg(:,7),RMSE_std(:,7));
xlabel('convergence threshold','Interpreter','latex')
ylabel('RMSE [m/s]','Interpreter','latex')
title({'Monte Carlo Study - ODR','cfar-800\_10Hz\_run0'},'Interpreter','latex')
hdl = legend('$v_x$','$v_y$','$v_z$');
set(hdl,'Interpreter','latex')

figure(15)
plot(odr_iter_avg(:,1), odr_iter_avg(:,2),'-o')
xlabel('convergence threshold','Interpreter','latex')
ylabel('ODR iterations','Interpreter','latex')
title({'Monte Carlo Study - ODR Iterations','cfar-800\_10Hz\_run0'},'Interpreter','latex')
hdl = legend('ODR iterations');
set(hdl,'Interpreter','latex')

    
return;

%% Save Data to CSV file

dlmwrite(strcat(filename,'.csv'), data, 'delimiter', ',', 'precision', '%.16f');
