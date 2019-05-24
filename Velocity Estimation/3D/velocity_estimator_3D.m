%% Header

%%% Filename:   velocity_estimator_3D.m
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

path     = '/home/carl/Data/subT/radar-rig/vicon_2019-05-08/';
filename = 'cfar-1280_10Hz_run0';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

%% Modify Radar Data

% define start and finish indices to remove unwanted data at start and end of test run
% start = 3;
% finish = 1078;  % flight with only  static targets
% finish = 1070;  % flight with dynamics

% start = 1;
% finish = 1103;

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
converge_thres = 0.005;

%% Get Vicon Body-Frame Velocities

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
% 
% % twist_linear_body = smoothdata(twist_linear_body,1);
% 
% % get euler angles
% euler_angles = quat2eul(orientation);
% 
% % plot ground truth body-frame velocities
% plot(ax_h(1), twist_time_second, twist_linear_body)
% hdl = legend(ax_h(1), '$v_x$','$v_y$','$v_z$');
% set(hdl,'Interpreter','latex','Location','best');
% xlim(ax_h(1),[0, twist_time_second(end)]);
% 
% % plot euler angles - to visualize pitch and roll
% plot(ax_h(2), pose_time_second, rad2deg(euler_angles(:,2:3)));
% hdl = legend(ax_h(2), 'pitch, $\phi$', 'roll, $\theta$');
% set(hdl,'Interpreter','latex','Location','best');
% % ylim(ax_h(2),[-20,20]);
% xlim(ax_h(2),[0, pose_time_second(end)]);
% 
% figure(3)
% plot(twist_time_second, vecnorm(twist_linear_body,2,2)); hold on;
% plot(twist_time_second(1:end-1), abs(diff(vecnorm(twist_linear_body,2,2))))
% ylabel('body-frame velocity vector norm','Interpreter','latex')
% xlabel('time [s]','Interpreter','latex');
% xlim([0, twist_time_second(end)]);

%% Implement Estimation Scheme

p = 3;  % dimension of velocity estimate vector
NScans = size(radar_doppler,1);

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
            
            if (Ntargets_inlier > 2) && (~isnan(model_mlesac(1)))
                % get target weights
                int_range_scaled = [max(intensity(inlier_idx')); int_range(2)];
                weights = odr_getWeights(intensity(inlier_idx'), sigma_vr, int_range_scaled);
                
                delta = normrnd(0,sigma_theta,[2*Ntargets_inlier,1]);
                [ model_odr_weighted, ~, cov_weighted, odr_iter(i) ] = ODR_3D( doppler(inlier_idx'), ...
                    azimuth(inlier_idx'), elevation(inlier_idx'), d, ...
                    model_mlesac, delta, weights, converge_thres );

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

%% Compute RMSE for various estimation schemes

% [RMSE_mlesac, error_mlesac] = getDopplerRMSE( vhat_MLESAC, radar_time_second, ...
%     twist_time_second, twist_linear_body, p);
% 
% [RMSE_odr_weighted, error_odr] = getDopplerRMSE( vhat_ODR_weighted, ...
%     radar_time_second, twist_time_second, twist_linear_body, p);
% 
% disp('RMSE_MLESAC = ')
% disp(RMSE_mlesac)
% disp('RMSE_ODR_weighted = ')
% disp(RMSE_odr_weighted)

%% Plot Data

cla(ax_h(5));
cla(ax_h(6));
cla(ax_h(7));

cla(ax_h(10));
cla(ax_h(11));
cla(ax_h(12));

cla(ax_h(16));
cla(ax_h(17));
cla(ax_h(18));

% plot MLESAC estimate
% plot(ax_h(5),twist_time_second,twist_linear_body(:,1),...
%     'color',colors(2,:),'LineWidth',1);
plot(ax_h(5),radar_time_second,vhat_MLESAC(:,1),'k','LineWidth',1);
% plot(ax_h(6),twist_time_second,twist_linear_body(:,2),...
%     'color',colors(2,:),'LineWidth',1);
plot(ax_h(6),radar_time_second,vhat_MLESAC(:,2),'k','LineWidth',1);
% plot(ax_h(7),twist_time_second,twist_linear_body(:,3),...
%     'color',colors(2,:),'LineWidth',1);
plot(ax_h(7),radar_time_second,vhat_MLESAC(:,3),'k','LineWidth',1);
% ylim(ax_h(6),[-1.5,1.5]); ylim(ax_h(7),[-1,1]);
xlim(ax_h(5), [0, radar_time_second(end)]);
xlim(ax_h(6), [0, radar_time_second(end)]);
xlim(ax_h(7), [0, radar_time_second(end)]);
% hdl = legend(ax_h(7),'Vicon system','MLESAC');
hdl = legend(ax_h(5),'MLESAC');
set(hdl,'Interpreter','latex','Location','northwest')

% plot weighted ODR estimate
% plot(ax_h(10),twist_time_second,twist_linear_body(:,1), ...
%     'color',colors(2,:),'LineWidth',1);
plot(ax_h(10),radar_time_second,vhat_ODR_weighted(:,1),'k','LineWidth',1);
% plot(ax_h(11),twist_time_second,twist_linear_body(:,2), ...
%     'color',colors(2,:),'LineWidth',1);
plot(ax_h(11),radar_time_second,vhat_ODR_weighted(:,2),'k','LineWidth',1);
% plot(ax_h(12),twist_time_second,twist_linear_body(:,3), ...
%     'color',colors(2,:),'LineWidth',1);
plot(ax_h(12),radar_time_second,vhat_ODR_weighted(:,3),'k','LineWidth',1);
% ylim(ax_h(10),[-1,1.5]); ylim(ax_h(11),[-1,1]);
xlim(ax_h(10), [0, radar_time_second(end)]);
xlim(ax_h(11), [0, radar_time_second(end)]);
xlim(ax_h(12), [0, radar_time_second(end)]);
% hdl = legend(ax_h(11),'Vicon system','weighted ODR');
hdl = legend(ax_h(10),'weighted ODR');
set(hdl,'Interpreter','latex','Location','northwest')

K = 10;
% plot weighted ODR + covariance bounds
plot(ax_h(16),radar_time_second,vhat_ODR_weighted(:,1),'k','LineWidth',1);
plot(ax_h(16),radar_time_second,vhat_ODR_weighted(:,1) + ...
    K*sigma_odr_weighted(:,1),'r--');
plot(ax_h(16),radar_time_second,vhat_ODR_weighted(:,1) - ...
    K*sigma_odr_weighted(:,1),'r--');
plot(ax_h(17),radar_time_second,vhat_ODR_weighted(:,2),'k','LineWidth',1);
plot(ax_h(17),radar_time_second,vhat_ODR_weighted(:,2) + ...
    K*sigma_odr_weighted(:,2),'r--');
plot(ax_h(17),radar_time_second,vhat_ODR_weighted(:,2) - ...
    K*sigma_odr_weighted(:,2),'r--');
plot(ax_h(18),radar_time_second,vhat_ODR_weighted(:,3),'k','LineWidth',1);
plot(ax_h(18),radar_time_second,vhat_ODR_weighted(:,3) + ...
    K*sigma_odr_weighted(:,3),'r--');
plot(ax_h(18),radar_time_second,vhat_ODR_weighted(:,3) - ...
    K*sigma_odr_weighted(:,3),'r--');
% ylim(ax_h(14),[-1,1.5]); ylim(ax_h(15),[-1,1]);
xlim(ax_h(16), [0, radar_time_second(end)]);
xlim(ax_h(17), [0, radar_time_second(end)]);
xlim(ax_h(18), [0, radar_time_second(end)]);
hdl = legend(ax_h(16),'weighted ODR','2$\sigma$ envelope');
set(hdl,'Interpreter','latex','Location','northwest')

figure(20)
plot(radar_time_second,vecnorm(vhat_MLESAC')); hold on;
ylabel('vector norm','Interpreter','latex')
yyaxis right
scatter(radar_time_second,targets(:,3),sz,'filled')
ylabel('inlier targets','Interpreter','latex')
xlim([0, radar_time_second(end)]);
xlabel('time [s]','Interpreter','latex');
title('MLESAC Velocity Vector Norm','Interpreter','latex');

figure(21)
plot(radar_time_second,vecnorm(vhat_ODR_weighted')); hold on;
ylabel('vector norm','Interpreter','latex')
yyaxis right
scatter(radar_time_second,targets(:,3),sz,'filled')
ylabel('inlier targets','Interpreter','latex')
xlim([0, radar_time_second(end)]);
xlabel('time [s]','Interpreter','latex');
title('Weighted ODR Velocity Vector Norm','Interpreter','latex');

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
plot(radar_time_second,sigma_odr_weighted(:,1)); hold on
plot(radar_time_second,sigma_odr_weighted(:,2))
plot(radar_time_second,sigma_odr_weighted(:,3))
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

dlmwrite(strcat(strcat(filename,'_2'),'.csv'), data, 'delimiter', ',', 'precision', '%.16f');
