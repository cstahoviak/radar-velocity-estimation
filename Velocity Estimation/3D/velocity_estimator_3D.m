%% Header

%%% Filename:   velocity_estimator_3D.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact

opts = optimset('display','off');   % for LSQNONLIN

% Task: To estimate the3D body-frame velocity of the sensor platfrom given
% input data from a single radar (/mmWaveDataHdl/RScan topic). The
% velocity estimation scheme takes the following approach:
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

% vehicle = 'jackal';
vehicle = 'quad';
ISRR = false;           % evaluating data from ISRR submission 

if strcmp(vehicle,'quad')
%     path = '/home/carl/Data/icra_2020/radar-quad/uas_flight_space/2019-09-11/';
    path = '/home/carl/Data/icra_2020/radar-quad/uas_flight_space/2019-09-13/';
elseif strcmp(vehicle,'jackal')
    path = '/home/carl/Data/subT/radar-rig/vicon_2019-05-08/';
else
    path = '';
end
filename = 'complex_light-max_run4';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

% print groundtruth statistics
vicon_stats = false;
gt1_stats   = false;

% does dataset include T265 VIO data?
t265 = true;

%% Modify Radar Data

% define start and finish indices to remove unwanted data at start and end of test run
% start = 1;
% finish = 1103;

% undo RVIZ plotting defaults
radar_y = -radar_y;
radar_z = -radar_z;

% calculate radar azimuth/elevation
radar_azimuth = atan(radar_y./radar_x);         % [rad]
radar_elevation = asin(radar_z./radar_range);   % [rad]

%% Create Figures

[ fig_h, ax_h ] = createVelocityEstimationPlots_3D();

% load colors for plotting
load('colors.mat');
sz = 4;     % scatter plot marker size

%% Define AIRE Filtering Constants

% set filtering thresholds
range_thres     = 0.3;      % [m]
azimuth_thres   = 85;       % [deg]
elevation_thres = 85;       % [deg]
intensity_thres = 1;        % [dB]
thresholds_AIRE = [azimuth_thres; intensity_thres; range_thres; elevation_thres];

norm_thresh = inf;   % used to reject 'bad' ODR estimates

%% Define MLESAC parameters

sampleSize  = 3;        % problem uniquely-determined for 2 targets
maxDistance = 0.15;     % only roughly tuned at this point

%% Define ODR parameters

max_intensity = max(max(radar_intensity));
min_intensity = min(min(radar_intensity));
int_range = [max_intensity; min_intensity];

load('1642_azimuth_bins.mat')
sigma_vr = 0.044;           % [m/s]
sigma_phi = sigma_theta;    % for now - need to conduct elev. bin study.
sigma = [sigma_theta; sigma_phi];

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

% scaling factor for step s - ODR_v5
s = 10*ones(sampleSize,1);

converge_thres = 0.0005;
max_iter = 50;
get_covar = true;

%% Get Vicon Body-Frame Velocities - Central Diff + Lowpass Filter

% 1. Apply Central Diff + Lowpass Filter to Vicon position data
% 2. Apply Central Diff + Lowpass Filter + Moving Avg Filter to Vicon position data
% 3. Vicon body-frame velocities on /vrpn_client_node/<object>/twist topic

% plot ground-truth position
plot(ax_h(1),pose_time_second,pose_position_x);
plot(ax_h(2),pose_time_second,pose_position_y);
plot(ax_h(3),pose_time_second,pose_position_z);
legend(ax_h(1),'Vicon System','Interpreter','latex');
xlim(ax_h(1),[0, pose_time_second(end)])
xlim(ax_h(2),[0, pose_time_second(end)])
xlim(ax_h(3),[0, pose_time_second(end)])

% NOTE: Vicon pose message orientation in N.W.U. frame
orientation = [pose_orientation_w, ... 
               pose_orientation_x, ... 
               pose_orientation_y, ...
               pose_orientation_z];
           
% get euler angles
euler_angles = quat2eul(orientation);
           
% 1. use central difference method (as opposed to diff) to calculate derivative
h = 0.01;   % vrpn system @ 100 Hz
velocity_x = central_diff(pose_position_x, h);
velocity_y = central_diff(pose_position_y, h);
velocity_z = central_diff(pose_position_z, h);
velocity = [velocity_x, velocity_y, velocity_z];

velocity_time_stamp = pose_time_stamp(2:end-1);
velocity_time_second = pose_time_second(2:end-1);

velocity_body = getBodyFrameVelocities( velocity, orientation, ...
        pose_time_stamp, velocity_time_stamp );

% 1. Map from NWU to NED coordinate frame
velocity_body(:,2) = -velocity_body(:,2);
velocity_body(:,3) = -velocity_body(:,3);
    
% 1. apply lowpass filtering to velocity data
sample_freq = 100;     % vrpn system @ 100 Hz
fpass = 0.12;
velocity_body = lowpass(velocity_body,fpass,sample_freq);

% 2. implement moving-average filtering
span = 5;
method = 'moving';

smoothed_vx = smooth(velocity_body(:,1),span,method);
smoothed_vy = smooth(velocity_body(:,2),span,method);
smoothed_vz = smooth(velocity_body(:,3),span,method);

velocity_body_smooth = [smoothed_vx, smoothed_vy, smoothed_vz];

% 3. get Vicon groundtruth ego-velocity (/vrpn_client_node/<object>/twist)
twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
twist_linear_body = getBodyFrameVelocities( twist_linear, orientation, ...
    pose_time_stamp, twist_time_stamp );

% 3. Map from NWU to NED coordinate frame
twist_linear_body(:,2) = -twist_linear_body(:,2);
twist_linear_body(:,3) = -twist_linear_body(:,3);

% plot Vicon groundtruth ego-velocity (/vrpn_client_node/<object>/twist)
plot(ax_h(4),twist_time_second,twist_linear_body(:,1))
plot(ax_h(5),twist_time_second,twist_linear_body(:,2))
plot(ax_h(6),twist_time_second,twist_linear_body(:,3))

% plot central diff + LP filter ego-velocity
plot(ax_h(4),velocity_time_second,velocity_body(:,1))
plot(ax_h(5),velocity_time_second,velocity_body(:,2))
plot(ax_h(6),velocity_time_second,velocity_body(:,3))

% plot central diff + LP + moving avg filter ego-velocity
plot(ax_h(4),velocity_time_second,velocity_body_smooth(:,1))
plot(ax_h(5),velocity_time_second,velocity_body_smooth(:,2))
plot(ax_h(6),velocity_time_second,velocity_body_smooth(:,3))
legend(ax_h(4),'Vicon System','Cntrl Diff + LP','Cntrl Diff + LP + Mov. Avg.', ...
    'Interpreter','latex')
xlim(ax_h(4),[0, twist_time_second(end)])
xlim(ax_h(5),[0, twist_time_second(end)])
xlim(ax_h(6),[0, twist_time_second(end)])

% figure(100)
% plot(twist_time_second, vecnorm(twist_linear_body,2,2)); hold on;
% plot(twist_time_second(1:end-1), abs(diff(vecnorm(twist_linear_body,2,2))))
% ylabel('body-frame velocity vector norm','Interpreter','latex')
% xlabel('time [s]','Interpreter','latex');
% xlim([0, twist_time_second(end)]); 

% plot euler angles - to visualize pitch and roll
plot(ax_h(7), pose_time_second, rad2deg(euler_angles(:,2:3)));
hdl = legend(ax_h(7), 'pitch, $\theta$', 'roll, $\phi$');
set(hdl,'Interpreter','latex','Location','northwest');
% ylim(ax_h(2),[-20,20]);
xlim(ax_h(7),[0, pose_time_second(end)]);

%% Get Andrew's Ground Truth Body-Frame Velocities

if ISRR
    gt_path = strcat(path,'processed_data/mat_files/');
    subdir   = 'ground_truth/';
    suffix   = '_gt';

    gt_file = strcat(gt_path,subdir,filename,suffix,filetype);
    load(gt_file);

    gt_position    = [gt_position_x, gt_position_y, gt_position_z];
    gt_orientation = [gt_orientation_w, gt_orientation_x, ...
                      gt_orientation_y, gt_orientation_z];
    gt_velocity    = [gt_velocity_x, gt_velocity_y, gt_velocity_z];

    % transfrom grouth-truth velocities
    gt_velocity_body = getBodyFrameVelocities( gt_velocity, gt_orientation, ...
        gt_time_stamp, gt_time_stamp );

    % get euler angles
    euler_gt = quat2eul(gt_orientation);

    % % plot ground truth body-frame velocities
    % plot(ax_h(1), gt_time_second, gt_velocity_body)
    % hdl = legend(ax_h(1), '$v_x$','$v_y$','$v_z$');
    % set(hdl,'Interpreter','latex','Location','best');
    % xlim(ax_h(1),[0, gt_time_second(end)]);
    % 
    % % plot euler angles - to visualize pitch and roll
    % plot(ax_h(2), gt_time_second, rad2deg(euler_gt(:,2:3)));
    % hdl = legend(ax_h(2), 'pitch, $\theta$', 'roll, $\phi$');
    % set(hdl,'Interpreter','latex','Location','best');
    % % ylim(ax_h(2),[-20,20]);
    % xlim(ax_h(2),[0, gt_time_second(end)]);
end

%% Tranform T265 Velocity into NED frame

odom_velocity_body = [-odom_velocity_linear_x, odom_velocity_linear_y, ...
    -odom_velocity_linear_z];

%% Implement Estimation Scheme

p = 3;  % dimension of velocity estimate vector
Nscans = size(radar_doppler,1);

% init estimate vectors
vhat_mlesac    = NaN*ones(Nscans,p);
vhat_lsqnonlin = NaN*ones(Nscans,p);
vhat_odr       = NaN*ones(Nscans,p);    % constant-weight ODR_v5
vhat_odr_w     = NaN*ones(Nscans,p);    % weighted ODR_v5

% init covariance matrices
sigma_odr   = NaN*ones(Nscans,p);       % constant-weight ODR_v5
sigma_odr_w = NaN*ones(Nscans,p);       % weighted ODR_v5

% data to save to csv file
data_save = NaN*ones(Nscans,10);  % [stamp, vx, vy, vz, sigsq_x, sigsq_y, sigsq_z, sig_xy, sig_xz, sig_yz]
data_save(:,1) = radar_time_stamp;

targets = NaN*ones(Nscans,3);   % [Ntargets, Ntargets_valid, Ntargets_inlier]
inliers = NaN*ones(Nscans,1);   % number of MLESAC inliers
odr_iter = NaN*ones(Nscans,2);  % number of iteratiosn required for ODR convergence
time = NaN*ones(Nscans,4);      % [mlesac, lsqnonlin, odr_v5, odr_v5_w]

for i=1:Nscans
    
    %%% ORTHOGONAL DISTANCE REGRESSION ON MLESAC INLIER SET %%%%%%%%%%%%%%%
    fprintf('RScan: %d', i);
    
    Ntargets = sum( ~isnan(radar_doppler(i,:)) );
    targets(i,1) = Ntargets;
    
    % remove NaNs and apply AIRE {angle, intensity, range, elevation} filtering
    tic
    data_AIRE = [radar_azimuth(i,:)', radar_intensity(i,:)', ...
                 radar_range(i,:)', radar_elevation(i,:)'];
    [ idx_AIRE ] = AIRE_filtering( data_AIRE, thresholds_AIRE);
    
    % redefine doppler, angle and intensity vectors to use below
    doppler   = radar_doppler(i,idx_AIRE)';
    azimuth   = radar_azimuth(i,idx_AIRE)';
    elevation = radar_elevation(i,idx_AIRE)';
    intensity = radar_intensity(i,idx_AIRE)';
    
    % number of valid targets per scan
    Ntargets_valid = sum(idx_AIRE);
    targets(i,2) = Ntargets_valid;
    
    if Ntargets_valid > 5
        % NOTE: a valid velocity estimate can only be derived for 3 or more
        % targets located at distinct azimuth bins

        % get MLESAC (Maximum Likelihood RANSAC) model and inlier set
        [ model_mlesac, inlier_idx ] = MLESAC_3D( doppler', azimuth', ...
            elevation', sampleSize, maxDistance );
        time(i,1) = toc;
        Ntargets_inlier = sum(inlier_idx);
        targets(i,3) = Ntargets_inlier;
        fprintf('\tNinliers = %d\n', Ntargets_inlier);

        % reject 'bad' estimates - do this for ODR too (eventually)
        if norm(model_mlesac) < norm_thresh
            vhat_mlesac(i,:) = -model_mlesac';
        elseif i > 1
            % use the previous estimate
            vhat_mlesac(i,:) = vhat_mlesac(i-1,:);
        else
            % do nothing
        end
        
        % concatenate ODR data
        data = [doppler(inlier_idx), azimuth(inlier_idx), ...
            elevation(inlier_idx)];
        
        % get constant-weight ODR estimate
        tic
        weights_const = (1/sigma_vr)*ones(Ntargets_inlier,1);
        [ model_odr, ~, cov, iter ] = ODR_v5( data, d, model_mlesac, ...
            sigma, weights_const, s, converge_thres, max_iter, get_covar );
        time(i,3) = toc;
        
        % get target weights
        tic
        int_range_scaled = [max(intensity(inlier_idx)); int_range(2)];
        weights = odr_getWeights(intensity(inlier_idx), sigma_vr, int_range_scaled);
        % % get weighted ODR estimate
        [ model_odr_w, ~, cov_w, iter_w ] = ODR_v5( data, d, model_mlesac, ...
            sigma, weights, s, converge_thres, max_iter, get_covar );
        time(i,4) = toc;

        % get standard deviations for vx vy, vz
        sigma_odr(i,:)    = sqrt(diag(cov));
        sigma_odr_w(i,:)  = sqrt(diag(cov_w));
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % ODR_v5 NOT working for non-constant weights. ODR_v5_test written
%         % to evaluate why this is happening. ODR_v5_test and the 'old'
%         % ODR_3D will be compared to find the bug in ODR_v5.
%         
%         % init delta vector such that both ODR_v5_test and ODR_3D share the
%         % same initialization
%         delta_theta = normrnd(0,sigma(1),[1,Ntargets_inlier]); 
%         delta_phi = normrnd(0,sigma(2),[1,Ntargets_inlier]);
%         delta0 = [delta_theta; delta_phi];
%         delta0 = delta0(:);
%         
%         % get weighted ODR_v5 estimate
%         tic
%         [ model_odr_w, ~, cov_w, iter_w ] = ODR_v5_test( data, d, model_mlesac, ...
%             sigma, weights, s, converge_thres, max_iter, get_covar, delta0 );
%         time(i,3) = toc;
%         
%         % get 'old' ODR_3D estimate as comparison
%         [ model_odr_3d, ~, cov_3d, iter_3d ] = ODR_3D( doppler(inlier_idx)', ...
%             azimuth(inlier_idx)', elevation(inlier_idx)', d, model_mlesac, ...
%             [delta_theta'; delta_phi'], weights, converge_thres );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % get LSQNONLIN (OLS) solution
        tic
        f = @(model) doppler_residual( model, data );
        x0 = ones(p,1);
        model_lsqnonlin = lsqnonlin(f,x0,[],[],opts);
        time(i,2) = toc;
        
        vhat_lsqnonlin(i,:) = -model_lsqnonlin';
        vhat_odr(i,:)       = -model_odr';
        vhat_odr_w(i,:)     = -model_odr_w';
        
    else
        warning('Fewer than 5 vald targets in scan\n')
        
        vhat_mlesac(i,:)    = NaN*ones(1,p);
        vhat_lsqnonlin(i,:) = NaN*ones(1,p);
        vhat_odr(i,:)       = NaN*ones(1,p);
        vhat_odr_w(i,:)     = NaN*ones(1,p);
    end
    
    data_save(i,2:end) = [vhat_odr_w(i,:), diag(cov_w)', ...
                             cov_w(1,2), cov_w(1,3), cov_w(2,3)];
end

% convert time to milliseconds
time = 1e3*time;

%% Compute Vicon RMSE Statistics

% stats = [vx_mean, vx_std, vx_min, vx_max;
%          vy_mean, vy_std, vy_min, vy_max;
%          vz_mean, vz_std, vz_min, vz_max];

if vicon_stats
    % Vicon - MLESAC RMSE statistics
    [rmse_mlesac_vicon, error_mlesac_vicon] = getRMSE( vhat_mlesac, ...
        radar_time_stamp, twist_linear_body, twist_time_stamp, p, norm_thresh);
    mlesac_vicon_stats = [sqrt(mean(error_mlesac_vicon.^2,1))', ...
        std(error_mlesac_vicon,1)', min(error_mlesac_vicon)', max(error_mlesac_vicon)'];

    % Vicon - LSQNONLIN RMSE statistics
    [rmse_lsqnonlin_vicon, error_lsqnonlin_vicon] = getRMSE( vhat_lsqnonlin, ...
        radar_time_stamp, twist_linear_body, twist_time_stamp, p, norm_thresh);
    lsqnonlin_vicon_stats = [sqrt(mean(error_lsqnonlin_vicon.^2,1))', ...
        std(error_lsqnonlin_vicon,1)', min(error_lsqnonlin_vicon)', max(error_lsqnonlin_vicon)'];

    % Vicon - ODR RMSE statistics
    [rmse_odr_vicon, error_odr_vicon] = getRMSE( vhat_odr, ...
        radar_time_stamp, twist_linear_body, twist_time_stamp, p, norm_thresh);
    odr_vicon_stats = [sqrt(mean(error_odr_vicon.^2,1))', ...
        std(error_odr_vicon,1)', min(error_odr_vicon)', max(error_odr_vicon)'];

    % Vicon - Weighted ODR RMSE statistics
    [rmse_odr_w_vicon, error_odr_w_vicon] = getRMSE( vhat_odr_w, ...
        radar_time_stamp, twist_linear_body, twist_time_stamp, p, norm_thresh);
    odr_w_vicon_stats = [sqrt(mean(error_odr_w_vicon.^2,1))', ...
        std(error_odr_w_vicon,1)', min(error_odr_w_vicon)', max(error_odr_w_vicon)'];

    fprintf('\nVicon Ego-Velocity Error -- Forward [m/s]\n')
    fprintf('\t\t mean\t std\t min\t max\n')
    fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_vicon_stats(1,1), ...
        mlesac_vicon_stats(1,2),mlesac_vicon_stats(1,3),mlesac_vicon_stats(1,4))
    fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_vicon_stats(1,1), ...
        lsqnonlin_vicon_stats(1,2),lsqnonlin_vicon_stats(1,3),lsqnonlin_vicon_stats(1,4))
    fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_vicon_stats(1,1), ...
        odr_vicon_stats(1,2),odr_vicon_stats(1,3),odr_vicon_stats(1,4))
    fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_vicon_stats(1,1), ...
        odr_w_vicon_stats(1,2),odr_w_vicon_stats(1,3),odr_w_vicon_stats(1,4))

    fprintf('\nVicon Ego-Velocity Error -- Lateral [m/s]\n')
    fprintf('\t\t mean\t std\t min\t max\n')
    fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_vicon_stats(2,1), ...
        mlesac_vicon_stats(2,2),mlesac_vicon_stats(2,3),mlesac_vicon_stats(2,4))
    fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_vicon_stats(2,1), ...
        lsqnonlin_vicon_stats(2,2),lsqnonlin_vicon_stats(2,3),lsqnonlin_vicon_stats(2,4))
    fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_vicon_stats(2,1), ...
        odr_vicon_stats(2,2),odr_vicon_stats(2,3),odr_vicon_stats(2,4))
    fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_vicon_stats(2,1), ...
        odr_w_vicon_stats(2,2),odr_w_vicon_stats(2,3),odr_w_vicon_stats(2,4))

    fprintf('\nVicon Ego-Velocity Error -- Vertical [m/s]\n')
    fprintf('\t\t mean\t std\t min\t max\n')
    fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_vicon_stats(3,1), ...
        mlesac_vicon_stats(3,2),mlesac_vicon_stats(3,3),mlesac_vicon_stats(3,4))
    fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_vicon_stats(3,1), ...
        lsqnonlin_vicon_stats(3,2),lsqnonlin_vicon_stats(3,3),lsqnonlin_vicon_stats(3,4))
    fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_vicon_stats(3,1), ...
        odr_vicon_stats(3,2),odr_vicon_stats(3,3),odr_vicon_stats(3,4))
    fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_vicon_stats(3,1), ...
        odr_w_vicon_stats(3,2),odr_w_vicon_stats(3,3),odr_w_vicon_stats(3,4))
end

%% Compute Groundtruth Method 1 RMSE Statistics

% GT1 - central diff + lowpass filter

% stats = [vx_mean, vx_std, vx_min, vx_max;
%          vy_mean, vy_std, vy_min, vy_max;
%          vz_mean, vz_std, vz_min, vz_max];

if gt1_stats
    % GT1 - MLESAC RMSE statistics
    [rmse_mlesac_gt1, error_mlesac_gt1] = getRMSE( vhat_mlesac, ...
        radar_time_stamp, velocity_body, velocity_time_stamp, p, norm_thresh);
    mlesac_gt1_stats = [sqrt(mean(error_mlesac_gt1.^2,1))', ...
        std(error_mlesac_gt1,1)', min(error_mlesac_gt1)', max(error_mlesac_gt1)'];

    % GT1 - LSQNONLIN RMSE statistics
    [rmse_lsqnonlin_gt1, error_lsqnonlin_gt1] = getRMSE( vhat_lsqnonlin, ...
        radar_time_stamp, velocity_body, velocity_time_stamp, p, norm_thresh);
    lsqnonlin_gt1_stats = [sqrt(mean(error_lsqnonlin_gt1.^2,1))', ...
        std(error_lsqnonlin_gt1,1)', min(error_lsqnonlin_gt1)', max(error_lsqnonlin_gt1)'];

    % GT1 - ODR RMSE statistics
    [rmse_odr_gt1, error_odr_gt1] = getRMSE( vhat_odr, ...
        radar_time_stamp, velocity_body, velocity_time_stamp, p, norm_thresh);
    odr_gt1_stats = [sqrt(mean(error_odr_gt1.^2,1))', ...
        std(error_odr_gt1,1)', min(error_odr_gt1)', max(error_odr_gt1)'];

    % GT1 - Weighted ODR RMSE statistics
    [rmse_odr_w_gt1, error_odr_w_gt1] = getRMSE( vhat_odr_w, ...
        radar_time_stamp, velocity_body, velocity_time_stamp, p, norm_thresh);
    odr_w_gt1_stats = [sqrt(mean(error_odr_w_gt1.^2,1))', ...
        std(error_odr_w_gt1,1)', min(error_odr_w_gt1)', max(error_odr_w_gt1)'];

    fprintf('\nGT1 Ego-Velocity Error -- Forward [m/s]\n')
    fprintf('\t\t mean\t std\t min\t max\n')
    fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_gt1_stats(1,1), ...
        mlesac_gt1_stats(1,2),mlesac_gt1_stats(1,3),mlesac_gt1_stats(1,4))
    fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_gt1_stats(1,1), ...
        lsqnonlin_gt1_stats(1,2),lsqnonlin_gt1_stats(1,3),lsqnonlin_gt1_stats(1,4))
    fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_gt1_stats(1,1), ...
        odr_gt1_stats(1,2),odr_gt1_stats(1,3),odr_gt1_stats(1,4))
    fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_gt1_stats(1,1), ...
        odr_w_gt1_stats(1,2),odr_w_gt1_stats(1,3),odr_w_gt1_stats(1,4))

    fprintf('\nGT1 Ego-Velocity Error -- Lateral [m/s]\n')
    fprintf('\t\t mean\t std\t min\t max\n')
    fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_gt1_stats(2,1), ...
        mlesac_gt1_stats(2,2),mlesac_gt1_stats(2,3),mlesac_gt1_stats(2,4))
    fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_gt1_stats(2,1), ...
        lsqnonlin_gt1_stats(2,2),lsqnonlin_gt1_stats(2,3),lsqnonlin_gt1_stats(2,4))
    fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_gt1_stats(2,1), ...
        odr_gt1_stats(2,2),odr_gt1_stats(2,3),odr_gt1_stats(2,4))
    fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_gt1_stats(2,1), ...
        odr_w_gt1_stats(2,2),odr_w_gt1_stats(2,3),odr_w_gt1_stats(2,4))

    fprintf('\nGT1 Ego-Velocity Error -- Vertical [m/s]\n')
    fprintf('\t\t mean\t std\t min\t max\n')
    fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_gt1_stats(3,1), ...
        mlesac_gt1_stats(3,2),mlesac_gt1_stats(3,3),mlesac_gt1_stats(3,4))
    fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_gt1_stats(3,1), ...
        lsqnonlin_gt1_stats(3,2),lsqnonlin_gt1_stats(3,3),lsqnonlin_gt1_stats(3,4))
    fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_gt1_stats(3,1), ...
        odr_gt1_stats(3,2),odr_gt1_stats(3,3),odr_gt1_stats(3,4))
    fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_gt1_stats(3,1), ...
        odr_w_gt1_stats(3,2),odr_w_gt1_stats(3,3),odr_w_gt1_stats(3,4))
end

%% Compute Groundtruth Method 2 RMSE Statistics

% GT2 - central diff + lowpass filter + moving avg filter

% stats = [vx_mean, vx_std, vx_min, vx_max;
%          vy_mean, vy_std, vy_min, vy_max;
%          vz_mean, vz_std, vz_min, vz_max];

% GT2 - MLESAC RMSE statistics
[rmse_mlesac_gt2, error_mlesac_gt2] = getRMSE( vhat_mlesac, ...
    radar_time_stamp, velocity_body_smooth, velocity_time_stamp, p, norm_thresh);
mlesac_gt2_stats = [sqrt(mean(error_mlesac_gt2.^2,1))', ...
    std(error_mlesac_gt2,1)', min(error_mlesac_gt2)', max(error_mlesac_gt2)'];

% GT2 - LSQNONLIN RMSE statistics
[rmse_lsqnonlin_gt2, error_lsqnonlin_gt2] = getRMSE( vhat_lsqnonlin, ...
    radar_time_stamp, velocity_body_smooth, velocity_time_stamp, p, norm_thresh);
lsqnonlin_gt2_stats = [sqrt(mean(error_lsqnonlin_gt2.^2,1))', ...
    std(error_lsqnonlin_gt2,1)', min(error_lsqnonlin_gt2)', max(error_lsqnonlin_gt2)'];

% GT2 - ODR RMSE statistics
[rmse_odr_gt2, error_odr_gt2] = getRMSE( vhat_odr, ...
    radar_time_stamp, velocity_body_smooth, velocity_time_stamp, p, norm_thresh);
odr_gt2_stats = [sqrt(mean(error_odr_gt2.^2,1))', ...
    std(error_odr_gt2,1)', min(error_odr_gt2)', max(error_odr_gt2)'];

% GT2 - Weighted ODR RMSE statistics
[rmse_odr_w_gt2, error_odr_w_gt2] = getRMSE( vhat_odr_w, ...
    radar_time_stamp, velocity_body_smooth, velocity_time_stamp, p, norm_thresh);
odr_w_gt2_stats = [sqrt(mean(error_odr_w_gt2.^2,1))', ...
    std(error_odr_w_gt2,1)', min(error_odr_w_gt2)', max(error_odr_w_gt2)'];

% GT2 - T265 VIO RMSE statistics
[rmse_vio_gt2, error_vio_gt2] = getRMSE( odom_velocity_body, ...
    odom_time_stamp, velocity_body_smooth, velocity_time_stamp, p, norm_thresh);
vio_gt2_stats = [sqrt(mean(error_vio_gt2.^2,1))', ...
    std(error_vio_gt2,1)', min(error_vio_gt2)', max(error_vio_gt2)'];

fprintf('\nGT2 Ego-Velocity Error -- Forward [m/s]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_gt2_stats(1,1), ...
    mlesac_gt2_stats(1,2),mlesac_gt2_stats(1,3),mlesac_gt2_stats(1,4))
fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_gt2_stats(1,1), ...
    lsqnonlin_gt2_stats(1,2),lsqnonlin_gt2_stats(1,3),lsqnonlin_gt2_stats(1,4))
fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_gt2_stats(1,1), ...
    odr_gt2_stats(1,2),odr_gt2_stats(1,3),odr_gt2_stats(1,4))
fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_gt2_stats(1,1), ...
    odr_w_gt2_stats(1,2),odr_w_gt2_stats(1,3),odr_w_gt2_stats(1,4))
if t265
    fprintf('T265 VIO\t %.4f\t %.4f\t %.4f\t %.4f\n',vio_gt2_stats(1,1), ...
        vio_gt2_stats(1,2),vio_gt2_stats(1,3),vio_gt2_stats(1,4))
end

fprintf('\nGT2 Ego-Velocity Error -- Lateral [m/s]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_gt2_stats(2,1), ...
    mlesac_gt2_stats(2,2),mlesac_gt2_stats(2,3),mlesac_gt2_stats(2,4))
fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_gt2_stats(2,1), ...
    lsqnonlin_gt2_stats(2,2),lsqnonlin_gt2_stats(2,3),lsqnonlin_gt2_stats(2,4))
fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_gt2_stats(2,1), ...
    odr_gt2_stats(2,2),odr_gt2_stats(2,3),odr_gt2_stats(2,4))
fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_gt2_stats(2,1), ...
    odr_w_gt2_stats(2,2),odr_w_gt2_stats(2,3),odr_w_gt2_stats(2,4))
if t265
    fprintf('T265 VIO\t %.4f\t %.4f\t %.4f\t %.4f\n',vio_gt2_stats(2,1), ...
        vio_gt2_stats(2,2),vio_gt2_stats(2,3),vio_gt2_stats(2,4))
end

fprintf('\nGT2 Ego-Velocity Error -- Vertical [m/s]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',mlesac_gt2_stats(3,1), ...
    mlesac_gt2_stats(3,2),mlesac_gt2_stats(3,3),mlesac_gt2_stats(3,4))
fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',lsqnonlin_gt2_stats(3,1), ...
    lsqnonlin_gt2_stats(3,2),lsqnonlin_gt2_stats(3,3),lsqnonlin_gt2_stats(3,4))
fprintf('ODR_v5\t\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_gt2_stats(3,1), ...
    odr_gt2_stats(3,2),odr_gt2_stats(3,3),odr_gt2_stats(3,4))
fprintf('Weighted ODR_v5\t %.4f\t %.4f\t %.4f\t %.4f\n',odr_w_gt2_stats(3,1), ...
    odr_w_gt2_stats(3,2),odr_w_gt2_stats(3,3),odr_w_gt2_stats(3,4))
if t265
    fprintf('T265 VIO\t %.4f\t %.4f\t %.4f\t %.4f\n',vio_gt2_stats(3,1), ...
        vio_gt2_stats(3,2),vio_gt2_stats(3,3),vio_gt2_stats(3,4))
end


%% Print Timing Statistics

time_stats = [mean(time,1)', std(time,1)', min(time)', max(time)'];

fprintf('\nAlgorithm Evaluation - Execution Time [milliseconds]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(1,1), ...
    time_stats(1,2),time_stats(1,3),time_stats(1,4))
fprintf('LSQNONLIN\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(2,1), ...
    time_stats(2,2),time_stats(2,3),time_stats(2,4))
fprintf('ODR_v5\t\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(3,1), ...
    time_stats(3,2),time_stats(3,3),time_stats(3,4))
fprintf('Weighted ODR_v5\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(4,1), ...
    time_stats(4,2),time_stats(4,3),time_stats(4,4))

%% Plot Data

% clear axes
for i=9:length(ax_h)
    cla(ax_h(i));
end

% use specific type of groundtruth method
% gt = twist_linear_body;
% gt = velocity_body;
gt = velocity_body_smooth;

% plot MLESAC ego-velocity estimate + groundtruth
h(1) = plot(ax_h(8),velocity_time_stamp,gt(:,1),'k','LineWidth',1);
plot(ax_h(9),velocity_time_stamp,gt(:,2),'k','LineWidth',1);
plot(ax_h(10),velocity_time_stamp,gt(:,3),'k','LineWidth',1);
h(2) = plot(ax_h(8),radar_time_stamp,vhat_mlesac(:,1),'color',colors(2,:));
plot(ax_h(9),radar_time_stamp,vhat_mlesac(:,2),'color',colors(2,:));
plot(ax_h(10),radar_time_stamp,vhat_mlesac(:,3),'color',colors(2,:));
xlim(ax_h(8),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(9),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(10),[velocity_time_stamp(1),velocity_time_stamp(end)])
if t265
    h(3) = plot(ax_h(8),odom_time_stamp,odom_velocity_body(:,1),'color',colors(4,:));
    plot(ax_h(9),odom_time_stamp,odom_velocity_body(:,2),'color',colors(4,:));
    plot(ax_h(10),odom_time_stamp,odom_velocity_body(:,3),'color',colors(4,:));
    
    legend(h,{'groundtruth','MLESAC','T265 VIO'},'Interpreter','latex')
else
    legend(h,{'groundtruth','MLESAC'},'Interpreter','latex')
end

% plot LSQNONLIN ego-velocity estimate + groundtruth
h(1) = plot(ax_h(11),velocity_time_stamp,gt(:,1),'k','LineWidth',1);
plot(ax_h(12),velocity_time_stamp,gt(:,2),'k','LineWidth',1);
plot(ax_h(13),velocity_time_stamp,gt(:,3),'k','LineWidth',1);
h(2) = plot(ax_h(11),radar_time_stamp,vhat_lsqnonlin(:,1),'color',colors(3,:));
plot(ax_h(12),radar_time_stamp,vhat_lsqnonlin(:,2),'color',colors(3,:));
plot(ax_h(13),radar_time_stamp,vhat_lsqnonlin(:,3),'color',colors(3,:));
xlim(ax_h(11),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(12),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(13),[velocity_time_stamp(1),velocity_time_stamp(end)])
if t265
    h(3) = plot(ax_h(11),odom_time_stamp,odom_velocity_body(:,1),'color',colors(4,:));
    plot(ax_h(12),odom_time_stamp,odom_velocity_body(:,2),'color',colors(4,:));
    plot(ax_h(13),odom_time_stamp,odom_velocity_body(:,3),'color',colors(4,:));
    
    legend(h,{'groundtruth','LSQNONLIN','T265 VIO'},'Interpreter','latex')
else
    legend(h,{'groundtruth','LSQNONLIN'},'Interpreter','latex')
end


% plot Const. Weight ODR_v5 ego-velocity estimate + groundtruth
h(1) = plot(ax_h(14),velocity_time_stamp,gt(:,1),'k','LineWidth',1);
plot(ax_h(15),velocity_time_stamp,gt(:,2),'k','LineWidth',1);
plot(ax_h(16),velocity_time_stamp,gt(:,3),'k','LineWidth',1);
h(2) = plot(ax_h(14),radar_time_stamp,vhat_odr(:,1),'color',colors(1,:));
plot(ax_h(15),radar_time_stamp,vhat_odr(:,2),'color',colors(1,:));
plot(ax_h(16),radar_time_stamp,vhat_odr(:,3),'color',colors(1,:));
xlim(ax_h(14),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(15),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(16),[velocity_time_stamp(1),velocity_time_stamp(end)])
if t265
    h(3) = plot(ax_h(14),odom_time_stamp,odom_velocity_body(:,1),'color',colors(4,:));
    plot(ax_h(15),odom_time_stamp,odom_velocity_body(:,2),'color',colors(4,:));
    plot(ax_h(16),odom_time_stamp,odom_velocity_body(:,3),'color',colors(4,:));
    
    legend(h,{'groundtruth','ODR','T265 VIO'},'Interpreter','latex')
else
    legend(h,{'groundtruth','ODR'},'Interpreter','latex')
end

% plot Weighted ODR_v5 ego-velocity estimate + groundtruth
h(1) = plot(ax_h(17),velocity_time_stamp,gt(:,1),'k','LineWidth',1);
plot(ax_h(18),velocity_time_stamp,gt(:,2),'k','LineWidth',1);
plot(ax_h(19),velocity_time_stamp,gt(:,3),'k','LineWidth',1);
h(2) = plot(ax_h(17),radar_time_stamp,vhat_odr_w(:,1),'color',colors(1,:));
plot(ax_h(18),radar_time_stamp,vhat_odr_w(:,2),'color',colors(1,:));
plot(ax_h(19),radar_time_stamp,vhat_odr_w(:,3),'color',colors(1,:));
xlim(ax_h(17),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(18),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(19),[velocity_time_stamp(1),velocity_time_stamp(end)])
if t265
    h(3) = plot(ax_h(17),odom_time_stamp,odom_velocity_body(:,1),'color',colors(4,:));
    plot(ax_h(18),odom_time_stamp,odom_velocity_body(:,2),'color',colors(4,:));
    plot(ax_h(19),odom_time_stamp,odom_velocity_body(:,3),'color',colors(4,:));
    
    legend(h,{'groundtruth','Weighted ODR','T265 VIO'},'Interpreter','latex')
else
    legend(h,{'groundtruth','Weighted ODR'},'Interpreter','latex')
end

% plot Const. Weight ODR_v5 + LSQNONLIN ego-velocity estimate + groundtruth
h(1) = plot(ax_h(20),velocity_time_stamp,gt(:,1),'k');
plot(ax_h(21),velocity_time_stamp,gt(:,2),'k');
plot(ax_h(22),velocity_time_stamp,gt(:,3),'k');
h(2) = plot(ax_h(20),radar_time_stamp,vhat_lsqnonlin(:,1),'color',colors(3,:));
plot(ax_h(21),radar_time_stamp,vhat_lsqnonlin(:,2),'color',colors(3,:));
plot(ax_h(22),radar_time_stamp,vhat_lsqnonlin(:,3),'color',colors(3,:));
h(3) = plot(ax_h(20),radar_time_stamp,vhat_odr_w(:,1),'color',colors(1,:));
plot(ax_h(21),radar_time_stamp,vhat_odr_w(:,2),'color',colors(1,:));
plot(ax_h(22),radar_time_stamp,vhat_odr_w(:,3),'color',colors(1,:));
xlim(ax_h(20),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(21),[velocity_time_stamp(1),velocity_time_stamp(end)])
xlim(ax_h(22),[velocity_time_stamp(1),velocity_time_stamp(end)])
if t265
    h(4) = plot(ax_h(20),odom_time_stamp,odom_velocity_body(:,1),'color',colors(4,:));
    plot(ax_h(21),odom_time_stamp,odom_velocity_body(:,2),'color',colors(4,:));
    plot(ax_h(22),odom_time_stamp,odom_velocity_body(:,3),'color',colors(4,:));
    
    legend(h,{'groundtruth','LSQNONLIN','Weighted ODR','T265 VIO'}, ...
        'Interpreter','latex')
else
    legend(h,{'groundtruth','LSQNONLIN','Weighted ODR'},'Interpreter','latex')
end

K = 10;
% plot weighted ODR + covariance bounds centered on estimate
plot(ax_h(23),radar_time_stamp,vhat_odr_w(:,1),'k','LineWidth',1);
plot(ax_h(23),radar_time_stamp,vhat_odr_w(:,1) + ...
    K*sigma_odr_w(:,1),'r--');
plot(ax_h(23),radar_time_stamp,vhat_odr_w(:,1) - ...
    K*sigma_odr_w(:,1),'r--');
plot(ax_h(24),radar_time_stamp,vhat_odr_w(:,2),'k','LineWidth',1);
plot(ax_h(24),radar_time_stamp,vhat_odr_w(:,2) + ...
    K*sigma_odr_w(:,2),'r--');
plot(ax_h(24),radar_time_stamp,vhat_odr_w(:,2) - ...
    K*sigma_odr_w(:,2),'r--');
plot(ax_h(25),radar_time_stamp,vhat_odr_w(:,3),'k','LineWidth',1);
plot(ax_h(25),radar_time_stamp,vhat_odr_w(:,3) + ...
    K*sigma_odr_w(:,3),'r--');
plot(ax_h(25),radar_time_second,vhat_odr_w(:,3) - ...
    K*sigma_odr_w(:,3),'r--');
xlim(ax_h(23), [radar_time_stamp(1), radar_time_stamp(end)]);
xlim(ax_h(24), [radar_time_stamp(1), radar_time_stamp(end)]);
xlim(ax_h(25), [radar_time_stamp(1), radar_time_stamp(end)]);
hdl = legend(ax_h(23),'weighted ODR\_v5','2$\sigma$ envelope');
set(hdl,'Interpreter','latex','Location','northwest')

% plot weighted ODR + covariance bounds centered at zero
plot(ax_h(26),radar_time_stamp,vhat_odr_w(:,1),'k','LineWidth',1);
plot(ax_h(26),radar_time_stamp,K*sigma_odr_w(:,1),'r--');
plot(ax_h(26),radar_time_stamp,-K*sigma_odr_w(:,1),'r--');
plot(ax_h(27),radar_time_stamp,vhat_odr_w(:,2),'k','LineWidth',1);
plot(ax_h(27),radar_time_stamp,K*sigma_odr_w(:,2),'r--');
plot(ax_h(27),radar_time_stamp,-K*sigma_odr_w(:,2),'r--');
plot(ax_h(28),radar_time_stamp,vhat_odr_w(:,3),'k','LineWidth',1);
plot(ax_h(28),radar_time_stamp,K*sigma_odr_w(:,3),'r--');
plot(ax_h(28),radar_time_stamp,-K*sigma_odr_w(:,3),'r--');
xlim(ax_h(26), [radar_time_stamp(1), radar_time_stamp(end)]);
xlim(ax_h(27), [radar_time_stamp(1), radar_time_stamp(end)]);
xlim(ax_h(28), [radar_time_stamp(1), radar_time_stamp(end)]);
hdl = legend(ax_h(26),'weighted ODR\_v5','2$\sigma$ envelope');
set(hdl,'Interpreter','latex','Location','northwest')

if t265
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Not using these plots anymore

figure(20)
plot(radar_time_stamp,vecnorm(vhat_mlesac')); hold on;
ylabel('vector norm','Interpreter','latex')
yyaxis right
scatter(radar_time_stamp,targets(:,3),sz,'filled')
ylabel('inlier targets','Interpreter','latex')
xlim([radar_time_stamp(1), radar_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
title('MLESAC Velocity Vector Norm','Interpreter','latex');

figure(21)
plot(radar_time_stamp,vecnorm(vhat_odr_w')); hold on;
ylabel('vector norm','Interpreter','latex')
yyaxis right
scatter(radar_time_stamp,targets(:,3),sz,'filled')
ylabel('inlier targets','Interpreter','latex')
xlim([radar_time_stamp(1), radar_time_stamp(end)]);
xlabel('time [s]','Interpreter','latex');
title('Weighted ODR Velocity Vector Norm','Interpreter','latex');

return;

figure(22)
scatter(radar_time_second,targets(:,1),sz,'filled'); hold on;
scatter(radar_time_second,targets(:,2),sz,'filled')
scatter(radar_time_second,targets(:,3),sz,'filled')
xlabel('time [s]','Interpreter','latex');
ylabel('number of targets','Interpreter','latex')
title('Targets in Radar Scan','Interpreter','latex');
hdl = legend('Total Targets','Valid Targets','Inlier Targets');
set(hdl,'Interpreter','latex','Location','best')

figure(24)
plot(radar_time_second,sigma_odr_w(:,1)); hold on
plot(radar_time_second,sigma_odr_w(:,2))
plot(radar_time_second,sigma_odr_w(:,3))
xlabel('time [s]','Interpreter','latex');
ylabel('standard deviation, $\sigma$ [m/s]','Interpreter','latex')
hdl = legend('$\sigma_{v_x}$','$\sigma_{v_y}$','$\sigma_{v_z}$');
set(hdl,'Interpreter','latex','Location','northwest')
xlim([0, radar_time_second(end)]);

% figure(25)
% plot(radar_time_second,vecnorm(error_mlesac(:,1:2),2,2)); hold on
% plot(radar_time_second,vecnorm(error_odr(:,1:2),2,2),'--');
% ylabel('estimation error, [m/s]','Interpreter','latex')
% yyaxis right
% plot(radar_time_second,odr_iter-1)
% ylabel('ODR iterations, k','Interpreter','latex')
% xlabel('time [s]','Interpreter','latex');
% hdl = legend('MLESAC error','ODR error','ODR iterations');
% set(hdl,'Interpreter','latex','Location','best')
% xlim([0, radar_time_second(end)]);
% 
% figure(26)
% plot(radar_time_second,mean(error_mlesac(:,1:2),2)); hold on
% plot(radar_time_second,mean(error_odr(:,1:2),2));
% ylabel('estimation error, [m/s]','Interpreter','latex')
% yyaxis right
% % plot(radar_time_second,odr_iter-1)
% ylabel('ODR iterations, k','Interpreter','latex')
% xlabel('time [s]','Interpreter','latex');
% hdl = legend('MLESAC error','ODR error','ODR iterations');
% set(hdl,'Interpreter','latex','Location','best')
% xlim([0, radar_time_second(end)]);

return;

%% Save Data to CSV file

dlmwrite(strcat(strcat(filename,'_2'),'.csv'), data_save, 'delimiter', ',', 'precision', '%.16f');
