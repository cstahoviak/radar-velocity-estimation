%% Header

%%% Filename:   montecarlo_ODR_2D.m
%%% Author:     Carl Stahoviak
%%% Created:    09/24/2019  

clear;
clc;
close all;

format compact

opts = optimset('display','off');   % for LSQNONLIN

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 2;                 % problem uniquely-determined for 3 targets
maxDistance = 0.15;          % only roughly tuned at this point

n = sampleSize;     % minimum number of points needed to fit the model
p = 0.95;           % probability of sampling at least one good inlier set
t = maxDistance;    % threshold to declare inlier/outlier

%% Define measurement parameters

load('1642_azimuth_bins.mat')

sigma_vr = 0.044;                   % [m/s]
sigma = [sigma_vr; sigma_theta];

% define ODR error variance ratios
d = sigma_vr/sigma_theta;

converge_thres = 0.0005;
max_iter = 50;
get_covar = false;

%% Define measurement parameters

% number of simulated targets
Ninliers = 100;
Noutliers = 35;
Ntargets = Ninliers + Noutliers;

min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]

sigma_vr_outlier = 1.5;     % [m/s]
sigma_outlier = [sigma_vr_outlier; sigma_theta];

%% Monte Carlo Study - Matlab MLESAC vs. LSQNONLIN vs. ODR_V1

mc_iter = 250;

rmse = NaN*ones(mc_iter,4);         % [mlesac, ols, odr_v1, odr_v2]
time = NaN*ones(mc_iter,4);         % [mlesac, ols, odr_v1, odr_v2]
inliers = NaN*ones(mc_iter,1);
odr_iter = NaN*ones(mc_iter,2);

for i=1:mc_iter
    
    fprintf('MC Iteration: %d\n', i);
    
    % define truth ego-velocity vector
    velocity = (max_vel-min_vel).*rand(2,1) + min_vel;
    
    % create noisy simulated INLIER radar measurements
    [~, ~, inlier_doppler, inlier_azimuth ] = getRadarMeasurements( ...
        Ninliers, velocity, radar_angle_bins, sigma );
    
    % create noisy simulated OUTLIER radar measurements
    [~, ~, outlier_doppler, outlier_azimuth ] = getRadarMeasurements( ...
        Noutliers, velocity, radar_angle_bins, sigma_outlier );

    % combine inlier and outlier data sets
    radar_doppler = [inlier_doppler; outlier_doppler];
    radar_azimuth = [inlier_azimuth; outlier_azimuth];
    
    % get Matlab MLESAC (Max. Likelihood RANSAC) model and inlier set
    tic
    [ model_mlesac, inlier_idx ] = MLESAC( radar_doppler', ...
        radar_azimuth', sampleSize, maxDistance );
    time(i,1) = toc;
    inliers(i) = sum(inlier_idx);
    
    % get 2D Orthogonal Distance Regression (ODR v1) estimate
    weights = (1/sigma_vr)*ones(inliers(i),1);
    data = [radar_doppler(inlier_idx), radar_azimuth(inlier_idx), ...
        zeros(inliers(i),1)];
    tic
    [ model_odr, ~, ~, iter ] = ODR_v1( data, d, model_mlesac, ...
        sigma(2), weights, converge_thres, max_iter, get_covar );
    time(i,3) = toc;
    
    % get Orthogonal Distance Regression (ODR v2) estimate - MLESAC seed
    tic
    [ model_odr2, ~, ~, iter2 ] = ODR_v2( data, d, model_mlesac, ...
        sigma(2), weights, converge_thres, max_iter, get_covar );
    time(i,4) = toc;
    
    % get LSQNONLIN (OLS) solution
    f = @(model) doppler_residual( model, data );
    x0 = ones(size(velocity,1),1);
    tic
    model_lsqnonlin = lsqnonlin(f,x0,[],[],opts);
    time(i,2) = toc;
    
    rmse(i,1) = sqrt(mean((velocity - model_mlesac).^2));
    rmse(i,2) = sqrt(mean((velocity - model_lsqnonlin).^2));
    rmse(i,3) = sqrt(mean((velocity - model_odr).^2)); 
    rmse(i,4) = sqrt(mean((velocity - model_odr2).^2));
    
    odr_iter(i,:) = [iter, iter2];
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
fprintf('LSQNONLIN\t %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(2,1), ...
    rmse_stats(2,2),rmse_stats(2,3),rmse_stats(2,4))
fprintf('2D ODR_v1\t %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(3,1), ...
    rmse_stats(3,2),rmse_stats(3,3),rmse_stats(3,4))
fprintf('2D ODR_v2\t %.4f\t %.4f\t %.4f\t %.4f\n',rmse_stats(4,1), ...
    rmse_stats(4,2),rmse_stats(4,3),rmse_stats(4,4))

fprintf('\nAlgorithm Evaluation - Execution Time [milliseconds]\n')
fprintf('\t\t mean\t std\t min\t max\n')
fprintf('Matlab MLESAC\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(1,1), ...
    time_stats(1,2),time_stats(1,3),time_stats(1,4))
fprintf('LSQNONLIN\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(2,1), ...
    time_stats(2,2),time_stats(2,3),time_stats(2,4))
fprintf('2D ODR_v1\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(3,1), ...
    time_stats(3,2),time_stats(3,3),time_stats(3,4))
fprintf('2D ODR_v2\t %.2f\t %.2f\t %.2f\t %.2f\n',time_stats(4,1), ...
    time_stats(4,2),time_stats(4,3),time_stats(4,4))

fprintf('\nAlgorithm Evaluation - ODR Iterations\n')
fprintf('\t\t mean\t std\t min\t max\t mode\n')
fprintf('3D ODR_v1\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n',iter_stats(1,1), ...
    iter_stats(1,2),iter_stats(1,3),iter_stats(1,4),iter_stats(1,5))
fprintf('3D ODR_v2\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n',iter_stats(2,1), ...
    iter_stats(2,2),iter_stats(2,3),iter_stats(2,4),iter_stats(2,5))

%% Generate PDF Data

Npts = 200;

% generate RMSE PDF data
x_rmse = linspace(0,rmse_stats(1,4),Npts);
pdf_type = 'Lognormal';
nbins = 20;
alpha = 0.2;    % histogram transparency, 0 < alpha < 1

pd_rmse_mlesac = fitdist(rmse(:,1),pdf_type);
pd_rmse_ols    = fitdist(rmse(:,2),pdf_type);
pd_rmse_odr    = fitdist(rmse(:,3),pdf_type);
pd_rmse_odr2   = fitdist(rmse(:,4),pdf_type);

pdf_rmse_mlesac = pdf(pd_rmse_mlesac,x_rmse);
pdf_rmse_ols = pdf(pd_rmse_ols,x_rmse);
pdf_rmse_odr = pdf(pd_rmse_odr,x_rmse);
pdf_rmse_odr2 = pdf(pd_rmse_odr2,x_rmse);

% generate Execution Time PDF data
x_time = linspace(0,time_stats(1,4),Npts);
x_time1 = linspace(0,time_stats(3,4),Npts);
x_time2 = linspace(0,time_stats(4,4),Npts);
pdf_type = 'Lognormal';

pd_time_mlesac = fitdist(time(:,1),pdf_type);
pd_time_ols    = fitdist(time(:,2),pdf_type);
pd_time_odr    = fitdist(time(:,3),'Normal');
pd_time_odr2   = fitdist(time(:,4),'Normal');

pdf_time_mlesac = pdf(pd_time_mlesac,x_time);
pdf_time_ols = pdf(pd_time_ols,x_time);
pdf_time_odr = pdf(pd_time_odr,x_time1);
pdf_time_odr2 = pdf(pd_time_odr2,x_time2);

%% Plot Results

load('colors.mat')

% define number of histogram bins
nbins_rmse = 20;
nbins_time = 20;
nbins_odr = 25;

% plot RMSE statistics 
figure(1)
hold on;
histogram(rmse(:,1),nbins_rmse,'Normalization','pdf','FaceColor', ...
    colors(2,:),'FaceAlpha',alpha)
histogram(rmse(:,2),nbins_rmse,'Normalization','pdf','FaceColor', ...
    colors(3,:),'FaceAlpha',alpha)
histogram(rmse(:,4),nbins_rmse,'Normalization','pdf','FaceColor', ...
    colors(1,:),'FaceAlpha',alpha)

plot(x_rmse,pdf_rmse_mlesac,'Color',colors(2,:),'LineWidth',2);
plot(x_rmse,pdf_rmse_ols,'Color',colors(3,:),'LineWidth',2);
plot(x_rmse,pdf_rmse_odr2,'Color',colors(1,:),'LineWidth',2);

title({'Algorithm Evaluation','Ego-Velocity RMSE'},'Interpreter','latex')
xlabel('RMSE [m/s]','Interpreter','latex')
ylabel('density','Interpreter','latex')
hdl = legend('Matlab MLSEAC','LSQNONLIN','ODR v2');
set(hdl,'Interpreter','latex')
xlim([0, rmse_stats(1,4)]);

% plot Execution Time statistics
figure(2)
hold on;
histogram(time(:,1),nbins_time,'Normalization','pdf','FaceColor', ...
    colors(2,:),'FaceAlpha',alpha)
histogram(time(:,2),nbins_time,'Normalization','pdf','FaceColor', ...
    colors(3,:),'FaceAlpha',alpha)
histogram(time(:,4),nbins_time,'Normalization','pdf','FaceColor', ...
    colors(1,:),'FaceAlpha',alpha)

plot(x_time,pdf_time_mlesac,'Color',colors(2,:),'LineWidth',2);
plot(x_time,pdf_time_ols,'Color',colors(3,:),'LineWidth',2);
plot(x_time2,pdf_time_odr2,'Color',colors(1,:),'LineWidth',2);

title({'Algorithm Evaluation','Execution Time'},'Interpreter','latex')
xlabel('execution time [ms]','Interpreter','latex')
ylabel('density','Interpreter','latex')
hdl = legend('Matlab MLSEAC','LSQNONLIN','ODR v2');
set(hdl,'Interpreter','latex')
xlim([0, time_stats(4,4)]);

% plot ODR Execution Time statistics
figure(3)
hold on;
histogram(time(:,3),nbins_odr,'Normalization','pdf','FaceColor', ...
    colors(4,:),'FaceAlpha',alpha)
histogram(time(:,4),nbins_odr,'Normalization','pdf','FaceColor', ...
    colors(1,:),'FaceAlpha',alpha)

plot(x_time1,pdf_time_odr,'Color',colors(4,:),'LineWidth',2);
plot(x_time2,pdf_time_odr2,'Color',colors(1,:),'LineWidth',2);

title({'2D ODR Algorithm Evaluation','Execution Time'},'Interpreter','latex')
xlabel('execution time [ms]','Interpreter','latex')
ylabel('density','Interpreter','latex')
hdl = legend('ODR v1','ODR v2');
set(hdl,'Interpreter','latex')
xlim([0, time_stats(3,4)]);

    