%% Header

%%% Filename:   montecarlo_ODR_3D.m
%%% Author:     Carl Stahoviak
%%% Created:    09/19/2019  

clear;
clc;
close all;

format compact

opts = optimset('display','off');   % for LSQNONLIN

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 3;                 % problem uniquely-determined for 3 targets
maxDistance = 0.15;          % only roughly tuned at this point

n = sampleSize;     % minimum number of points needed to fit the model
p = 0.95;           % probability of sampling at least one good inlier set
t = maxDistance;    % threshold to declare inlier/outlier

%% Define ODR Parameters

load('1642_azimuth_bins.mat')

sigma_vr = 0.044;                   % [m/s]
sigma_phi = sigma_theta;            % [rad]
sigma = [sigma_theta; sigma_phi];

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

converge_thres = 0.0005;
max_iter = 50;
get_covar = false;

%% Define measurement parameters

% generate simulated pointcloud
type = 'points';

% number of simulated targets
Ninliers = 50;
Noutliers = 35;
Ntargets = Ninliers + Noutliers;

min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]

sigma_vr_outlier = 1.5;     % [m/s]

%% Monte Carlo Study - Matlab MLESAC vs. LSQNONLIN vs. ODR_V1

mc_iter = 250;

rmse = NaN*ones(mc_iter,3);         % [mlesac, ols, odr]
time = NaN*ones(mc_iter,3);         % [mlesac, ols, odr]
inliers = NaN*ones(mc_iter,1);
odr_iter = NaN*ones(mc_iter,1);

for i=1:mc_iter
    
    fprintf('MC Iteration: %d\n', i);
    
    % define truth ego-velocity vector
    velocity = (max_vel-min_vel).*rand(3,1) + min_vel;
    
    % create noisy simulated INLIER radar measurements
    [~, ~, ~, inlier_doppler, inlier_azimuth, inlier_elevation] = ...
        getRadarMeasurements_3D( Ninliers, velocity, radar_angle_bins, sigma_vr, type );
    
    % create noisy simulated OUTLIER radar measurements
    [~, ~, ~, outlier_doppler, outlier_azimuth, outlier_elevation] = ...
        getRadarMeasurements_3D( Noutliers, velocity, radar_angle_bins, sigma_vr_outlier, type );

    % combine inlier and outlier data sets
    radar_doppler = [inlier_doppler; outlier_doppler];
    radar_azimuth = [inlier_azimuth; outlier_azimuth];
    radar_elevation = [inlier_elevation; outlier_elevation];

    radar_data = [(1:Ntargets)', radar_doppler, radar_azimuth, radar_elevation];
    
    % get Matlab MLESAC (Max. Likelihood RANSAC) model and inlier set
    tic
    [ model_mlesac, inlier_idx ] = MLESAC_3D( radar_doppler', ...
        radar_azimuth', radar_elevation', sampleSize, maxDistance );
    time(i,1) = toc;
    inliers(i) = sum(inlier_idx);
    
    % get LSQNONLIN (OLS) solution
    f = @(model) doppler_residual( model, radar_azimuth(inlier_idx), ...
        radar_elevation(inlier_idx), radar_doppler(inlier_idx));
    x0 = ones(size(velocity,1),1);
    tic
    model_lsqnonlin = lsqnonlin(f,x0,[],[],opts);
    time(i,2) = toc;
    
    % get 3D Orthogonal Distance Regression (ODR) estimate
    tic
    weights = (1/sigma_vr)*ones(inliers(i),1);
    data = [radar_doppler(inlier_idx), radar_azimuth(inlier_idx), ...
        radar_elevation(inlier_idx)];
    [ model_odr, beta, cov, iter ] = ODR_v1( data, d, model_mlesac, ...
        sigma, weights, converge_thres, max_iter, get_covar );
    time(i,3) = toc;
    
    rmse(i,1) = sqrt(mean((velocity - model_mlesac).^2));
    rmse(i,2) = sqrt(mean((velocity - model_lsqnonlin).^2));
    rmse(i,3) = sqrt(mean((velocity - model_odr).^2)); 
end

% convert time to milliseconds
time = 1e3*time;

%% Compute Statistics

rmse_mean  = mean(rmse,1)';
rmse_sigma = std(rmse,1)';

time_mean  = mean(time,1)';
time_sigma = std(time,1)';

fprintf('\nAlgorithm Evaluation - Ego-Velocity RMSE\n')
fprintf('\t\t mean\t std. dev.\n')
fprintf('Matlab MLESAC\t %.4f\t %.4f\n', rmse_mean(1), rmse_sigma(1))
fprintf('LSQNONLIN (OLS)\t %.4f\t %.4f\n', rmse_mean(2), rmse_sigma(2))
fprintf('ODR_v1\t\t %.4f\t %.4f\n', rmse_mean(3), rmse_sigma(3))

fprintf('\nAlgorithm Evaluation - Execution Time\n')
fprintf('\t\t mean\t\t std. dev.\n')
fprintf('Matlab MLESAC\t %.4f\t\t %.4f\n', time_mean(1), time_sigma(1))
fprintf('LSQNONLIN (OLS)\t %.4f\t\t %.4f\n', time_mean(2), time_sigma(2))
fprintf('ODR_v1\t\t %.4f\t %.4f\n', time_mean(3), time_sigma(3))

%% Plot Results

load('colors.mat')

x_rmse = 0:0.0005:0.1;
pdf_type = 'Lognormal';
nbins = 20;
alpha = 0.2;    % histogram transparency, 0 < alpha < 1

pd_rmse_mlesac = fitdist(rmse(:,1),pdf_type);
pd_rmse_ols    = fitdist(rmse(:,2),pdf_type);
pd_rmse_odr    = fitdist(rmse(:,3),pdf_type);

pdf_rmse_mlesac = pdf(pd_rmse_mlesac,x_rmse);
pdf_rmse_ols = pdf(pd_rmse_ols,x_rmse);
pdf_rmse_odr = pdf(pd_rmse_odr,x_rmse);

% plot RMSE statistics 
figure(1)
hold on;
histogram(rmse(:,1),nbins,'Normalization','pdf','FaceColor',colors(2,:),'FaceAlpha',alpha)
histogram(rmse(:,2),nbins,'Normalization','pdf','FaceColor',colors(3,:),'FaceAlpha',alpha)
histogram(rmse(:,3),nbins,'Normalization','pdf','FaceColor',colors(1,:),'FaceAlpha',alpha)

plot(x_rmse,pdf_rmse_mlesac,'Color',colors(2,:),'LineWidth',2);
plot(x_rmse,pdf_rmse_ols,'Color',colors(3,:),'LineWidth',2);
plot(x_rmse,pdf_rmse_odr,'Color',colors(1,:),'LineWidth',2);

title('Algorithm Evaluation -- Ego-Velocity RMSE','Interpreter','latex')
xlabel('RMSE [m/s]','Interpreter','latex')
ylabel('density','Interpreter','latex')
hdl = legend('Matlab MLSEAC','LSQNONLIN','ODR');
set(hdl,'Interpreter','latex')

% plot execution time statistics 

    