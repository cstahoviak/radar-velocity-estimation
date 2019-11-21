%% Header

%%% Filename:   montecarlo_mlesac_odr3D.m
%%% Author:     Carl Stahoviak
%%% Created:    10/18/2019  

clear;
clc;
close all;

format compact

opts = optimset('display','off');   % for LSQNONLIN

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 3;              % problem uniquely-determined for 3 targets
maxDistance = 0.15;          % only roughly tuned at this point

n = sampleSize;     % minimum number of points needed to fit the model
p = 0.95;           % probability of sampling at least one good inlier set
t = maxDistance;    % threshold to declare inlier/outlier
mlesac_max_iter = 50;

%% Define ODR Parameters

load('1642_azimuth_bins.mat')

sigma_vr = 0.044;                   % [m/s]
sigma_phi = sigma_theta;            % [rad]
sigma = [sigma_theta; sigma_phi];

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

% scaling factor for step s - ODR_v5
s = 10*ones(sampleSize,1);

converge_thres = 0.00025;
max_iter = 10;
get_covar = true;

%% Define measurement parameters

% generate simulated pointcloud
type = 'points';

% number of simulated targets
Ninliers = 125;
Noutliers = 35;
Ntargets = Ninliers + Noutliers;

min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]

sigma_vr_outlier = 1.5;     % [m/s]

%% Monte Carlo Study - Matlab MLESAC vs. LSQNONLIN vs. ODR

mc_iter = 500;

rmse = NaN*ones(mc_iter,4);     % [mlesac, d-mlesac, odr_v5-1, odr_v5-2]
time = NaN*ones(mc_iter,4);     % [mlesac, d-mlesac, odr_v5-1, odr_v5-2]
inliers = NaN*ones(mc_iter,2);
odr_iter = NaN*ones(mc_iter,2);

for i=1:mc_iter
    
    fprintf('MC Iteration: %d\n', i);
    
    % define truth ego-velocity vector
    velocity = (max_vel-min_vel).*rand(sampleSize,1) + min_vel;
    
    % create noisy simulated INLIER radar measurements
    [~, ~, ~, inlier_doppler, inlier_azimuth, inlier_elevation] = ...
        getRadarMeasurements_3D( Ninliers, velocity, radar_angle_bins, ...
        sigma_vr, sigma, type );
    
    % create noisy simulated OUTLIER radar measurements
    [~, ~, ~, outlier_doppler, outlier_azimuth, outlier_elevation] = ...
        getRadarMeasurements_3D( Noutliers, velocity, radar_angle_bins, ...
        sigma_vr_outlier, sigma, type );

    % combine inlier and outlier data sets
    radar_doppler = [inlier_doppler; outlier_doppler];
    radar_azimuth = [inlier_azimuth; outlier_azimuth];
    radar_elevation = [inlier_elevation; outlier_elevation];
    
    % get Matlab MLESAC (Max. Likelihood RANSAC) model and inlier set
    tic
    [ model_mlesac, inlier_idx ] = MLESAC_3D( radar_doppler, ...
        radar_azimuth, radar_elevation, sampleSize, maxDistance );
    time(i,1) = toc;
    inliers(i,1) = sum(inlier_idx);
    
    % get Doppler MLESAC (Max. Likelihood RANSAC) model and inlier set
    tic
    data = [radar_doppler, radar_azimuth, radar_elevation];
    [ model_mlesac2, inlier_idx2, ~ ] = doppler_mlesac( data, n, p, t, ...
        mlesac_max_iter, sigma_vr);
    time(i,2) = toc;
    inliers(i,2) = sum(inlier_idx2);
    
    % get Orthogonal Distance Regression (ODR_v5) estimate
    tic
    weights = (1/sigma_vr)*ones(inliers(i,1),1);
    data = [radar_doppler(inlier_idx), radar_azimuth(inlier_idx), ...
        radar_elevation(inlier_idx)];
    [ model_odr5_11, ~, ~, iter1 ] = ODR_v5( data, d, model_mlesac, ...
        sigma, weights, s, converge_thres, max_iter, get_covar );
    time(i,3) = toc;
    
    % get Orthogonal Distance Regression (ODR_v5) estimate
    tic
    weights = (1/sigma_vr)*ones(inliers(i,2),1);
    data = [radar_doppler(inlier_idx2), radar_azimuth(inlier_idx2), ...
        radar_elevation(inlier_idx2)];
    [ model_odr5_2, ~, ~, iter2 ] = ODR_v5( data, d, model_mlesac2, ...
        sigma, weights, s, converge_thres, max_iter, get_covar );
    time(i,4) = toc;

    % compute RMSE statistics
    rmse(i,1) = sqrt(mean((velocity - model_mlesac).^2));
    rmse(i,2) = sqrt(mean((velocity - model_mlesac2).^2));
    rmse(i,3) = sqrt(mean((velocity - model_odr5_11).^2));
    rmse(i,4) = sqrt(mean((velocity - model_odr5_2).^2));
    
    odr_iter(i,:) = [iter1, iter2];
end

% convert time to milliseconds
time = 1e3*time;

%% Compute Statistics

rmse_stats = [mean(rmse,1)', std(rmse,1)', min(rmse)', max(rmse)'];
time_stats = [mean(time,1)', std(time,1)', min(time)', max(time)'];
iter_stats = [mean(odr_iter,1)', std(odr_iter,1)', min(odr_iter)', ...
    max(odr_iter)', mode(odr_iter,1)'];

fprintf('\nAlgorithm Evaluation - Ego-Velocity RMSE [m/s]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(1,1), ...
    rmse_stats(1,2),rmse_stats(1,3),rmse_stats(1,4))
fprintf('CCS MLESAC\t %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(2,1), ...
    rmse_stats(2,2),rmse_stats(2,3),rmse_stats(2,4))
fprintf('ODR (M-mlesac)\t %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(3,1), ...
    rmse_stats(3,2),rmse_stats(3,3),rmse_stats(3,4))
fprintf('ODR (CCS-mlesac) %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(4,1), ...
    rmse_stats(4,2),rmse_stats(4,3),rmse_stats(4,4))

fprintf('\nAlgorithm Evaluation - Execution Time [milliseconds]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(1,1), ...
    time_stats(1,2),time_stats(1,3),time_stats(1,4))
fprintf('CCS MLESAC\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(2,1), ...
    time_stats(2,2),time_stats(2,3),time_stats(2,4))
fprintf('ODR (M-mlesac)\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(3,1), ...
    time_stats(3,2),time_stats(3,3),time_stats(3,4))
fprintf('ODR (CCS-mlesac) %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(4,1), ...
    time_stats(4,2),time_stats(4,3),time_stats(4,4))

fprintf('\nAlgorithm Evaluation - ODR Iterations\n')
fprintf('\t\t mean\t std\t min\t max\t mode\n')
fprintf('ODR (M-mlesac)\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n',iter_stats(1,1), ...
    iter_stats(1,2),iter_stats(1,3),iter_stats(1,4),iter_stats(1,5))
fprintf('ODR (CCS-mlesac) %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n',iter_stats(2,1), ...
    iter_stats(2,2),iter_stats(2,3),iter_stats(2,4),iter_stats(2,5))

%% Generate PDF Data

Npts = 200;

% generate RMSE PDF data
pdf_type = 'Lognormal';
nbins = 20;
alpha = 0.2;    % histogram transparency, 0 < alpha < 1

x_rmse_mlesac    = linspace(0,rmse_stats(1,4),Npts);
x_rmse_mlesac2   = linspace(0,rmse_stats(2,4),Npts);
x_rmse_odr5_1    = linspace(0,rmse_stats(3,4),Npts);
x_rmse_odr5_2    = linspace(0,rmse_stats(4,4),Npts);

pd_rmse_mlesac  = fitdist(rmse(:,1),'Normal');
pd_rmse_mlesac2 = fitdist(rmse(:,2),'Normal');
pd_rmse_odr5_1  = fitdist(rmse(:,3),pdf_type);
pd_rmse_odr5_2  = fitdist(rmse(:,4),pdf_type);

pdf_rmse_mlesac  = pdf(pd_rmse_mlesac,x_rmse_mlesac);
pdf_rmse_mlesac2 = pdf(pd_rmse_mlesac2,x_rmse_mlesac2);
pdf_rmse_odr5_1  = pdf(pd_rmse_odr5_1,x_rmse_odr5_1);
pdf_rmse_odr5_2  = pdf(pd_rmse_odr5_2,x_rmse_odr5_2);

% generate Execution Time PDF data
x_time_m1  = linspace(0,time_stats(1,4),Npts);
x_time_m2  = linspace(0,time_stats(2,4),Npts);
x_time5_1  = linspace(0,time_stats(3,4),Npts);
x_time5_2  = linspace(0,time_stats(4,4),Npts);
pdf_type = 'Lognormal';

pd_time_mlesac  = fitdist(time(:,1),pdf_type);
pd_time_mlesac2 = fitdist(time(:,2),pdf_type);
pd_time_odr5_1  = fitdist(time(:,3),pdf_type);
pd_time_odr5_2  = fitdist(time(:,4),pdf_type);

pdf_time_mlesac  = pdf(pd_time_mlesac,x_time_m1);
pdf_time_mlesac2 = pdf(pd_time_mlesac2,x_time_m2);
pdf_time_odr5_1    = pdf(pd_time_odr5_1,x_time5_1);
pdf_time_odr5_2    = pdf(pd_time_odr5_2,x_time5_2);

%% Plot Results

load('colors.mat')

% define number of histogram bins
nbins_rmse = 20;
nbins_time = 20;
nbins_odr = 25;

width_rmse = 0.005;
width_time = 0.25;

% plot RMSE statistics 
figure(1)
hold on;
histogram(rmse(:,1),'BinWidth',width_rmse,'Normalization','pdf', ...
    'FaceColor',colors(2,:),'FaceAlpha',alpha)
histogram(rmse(:,2),'BinWidth',width_rmse,'Normalization','pdf', ...
    'FaceColor',colors(4,:),'FaceAlpha',alpha)
histogram(rmse(:,3),'BinWidth',width_rmse,'Normalization','pdf', ...
    'FaceColor',colors(1,:),'FaceAlpha',alpha)
histogram(rmse(:,4),'BinWidth',width_rmse,'Normalization','pdf', ...
    'FaceColor',colors(3,:),'FaceAlpha',alpha)

plot(x_rmse_mlesac,pdf_rmse_mlesac,'Color',colors(2,:),'LineWidth',2);
plot(x_rmse_mlesac2,pdf_rmse_mlesac2,'Color',colors(4,:),'LineWidth',2);
plot(x_rmse_odr5_1,pdf_rmse_odr5_1,'Color',colors(1,:),'LineWidth',2);
plot(x_rmse_odr5_2,pdf_rmse_odr5_2,'Color',colors(3,:),'LineWidth',2);

title({'Algorithm Evaluation','Ego-Velocity RMSE'},'Interpreter','latex')
xlabel('RMSE [m/s]','Interpreter','latex')
ylabel('density','Interpreter','latex')
hdl = legend('Matlab MLSEAC','CCS MLESAC','ODR\_v5 - M-mlesac Seed', ...
    'ODR\_v5 - CCS-mlesac Seed');
set(hdl,'Interpreter','latex')
xlim([0, rmse_stats(2,4)]);

% plot Execution Time statistics
figure(2)
hold on;
histogram(time(:,1),'BinWidth',width_time,'Normalization','pdf', ...
    'FaceColor',colors(2,:),'FaceAlpha',alpha)
histogram(time(:,2),'BinWidth',width_time,'Normalization','pdf', ...
    'FaceColor',colors(4,:),'FaceAlpha',alpha)
histogram(time(:,3),'BinWidth',width_time,'Normalization','pdf', ...
    'FaceColor',colors(1,:),'FaceAlpha',alpha)
histogram(time(:,4),'BinWidth',width_time,'Normalization','pdf', ...
    'FaceColor',colors(3,:),'FaceAlpha',alpha)

plot(x_time_m1,pdf_time_mlesac,'Color',colors(2,:),'LineWidth',2);
plot(x_time_m2,pdf_time_mlesac2,'Color',colors(4,:),'LineWidth',2);
plot(x_time5_1,pdf_time_odr5_1,'Color',colors(1,:),'LineWidth',2);
plot(x_time5_2,pdf_time_odr5_2,'Color',colors(3,:),'LineWidth',2);

title({'Algorithm Evaluation','Execution Time'},'Interpreter','latex')
xlabel('execution time [ms]','Interpreter','latex')
ylabel('density','Interpreter','latex')
hdl = legend('Matlab MLSEAC','CCS MLESAC','ODR\_v5 - M-mlesac Seed', ...
    'ODR\_v5 - CCS-mlesac Seed');
set(hdl,'Interpreter','latex')
xlim([0, time_stats(4,4)]);

