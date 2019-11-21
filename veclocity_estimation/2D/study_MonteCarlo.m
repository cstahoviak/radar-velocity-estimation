%% Header

%%% Filename:   MonteCarloStudy.m
%%% Author:     Carl Stahoviak
%%% Created:    03/04/2019  

clear;
clc;
close all;

format compact

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 2;             % problem uniquely-determined for 2 targets
maxDistance = 0.1;          % only roughly tuned at this point
conditionNum_thres = 100;

%% Define measurement parameters

% number of simulated targets
Ntargets = 75;

sigma_vr = 0.044;     % [m/s]
load('radar_angle_bins.mat')

% define error variance ratio, d
d = ones(Ntargets,1)*(sigma_vr/sigma_theta);

%% Monte Carlo Study

% number of Monte Carlo interations
MCiter = 50;

% simulated 'true' platform velocity
min_vel = -2.5;     % [m/s]
max_vel = 2.5;      % [m/s]

RMSE_bruteforce     = zeros(MCiter,1);
RMSE_mlesac         = zeros(MCiter,1);
RMSE_odr_mlesac     = zeros(MCiter,1);
RMSE_odr_bruteforce = zeros(MCiter,1);

for i=1:MCiter
    
    fprintf('Monte Carlo interation: %d\n', i);
    
    % generate true velocity profile
    velocity = (max_vel-min_vel).*rand(2,1) + min_vel

    % create noisy simulated radar measurements
    [true_angle, true_doppler, radar_angle, radar_doppler ] = ...
        getRadarMeasurements( Ntargets, velocity, radar_angle_bins, sigma_vr );

    %%% Implement Estimation Schemes

    % get 'brute force' estimate of forward/lateral body-frame vel.
    [ model_bruteforce, vhat_all ] = getBruteForceEstimate( radar_doppler', ...
        radar_angle', conditionNum_thres);
    fprintf('Brute-Force  Velocity Profile Estimation\n');
    disp(model_bruteforce)

    % get MLESAC (M-estimator RANSAC) model and inlier set
    [ model_mlesac, inlier_idx ] = MLESAC( radar_doppler', ...
        radar_angle', sampleSize, maxDistance, conditionNum_thres );
    fprintf('MLESAC Velocity Profile Estimation\n');
    disp(model_mlesac)
    fprintf('MLESAC Number of Inliers\n');
    disp(sum(inlier_idx));

    % get Orthogonal Distance Regression (ODR) estimate - MLESAC seed
    weights = (1/sigma_vr)*ones(Ntargets,1);
    delta = normrnd(0,sigma_theta,[Ntargets,1]);
    [ model_odr1, beta1 ] = ODR( radar_angle', radar_doppler', d, ...
        model_mlesac, delta, weights );
    fprintf('ODR Velocity Profile Estimation\n');
    disp(model_odr1)

    % get Orthogonal Distance Regression (ODR) estimate - MLESAC seed
    weights = (1/sigma_vr)*ones(Ntargets,1);
    delta = normrnd(0,sigma_theta,[Ntargets,1]);
    [ model_odr2, beta2 ] = ODR( radar_angle', radar_doppler', d, ...
        model_mlesac, delta, weights );
    fprintf('ODR Velocity Profile Estimation\n');
    disp(model_odr2)

    RMSE_bruteforce(i)     = sqrt(mean((velocity - model_bruteforce).^2));
    RMSE_mlesac(i)         = sqrt(mean((velocity - model_mlesac).^2));
    RMSE_odr_mlesac(i)     = sqrt(mean((velocity - model_odr1).^2));
    RMSE_odr_bruteforce(i) = sqrt(mean((velocity - model_odr2).^2));

end

fprintf('Average RMSE values\n')

%% Plot figures

sz = 8;

close ALL;

figure(1)
plot(RMSE_bruteforce); hold on;
plot(RMSE_mlesac);
plot(RMSE_odr_mlesac,'LineWidth',2);
plot(RMSE_odr_bruteforce);
hdl = legend('brute-force','MLESAC', ...
    'ODR - MLESAC seed','ODR - brute-force seed');
set(hdl,'Interpreter','latex','Location','best')

figure(2)
idx = 1:MCiter;
scatter(idx, RMSE_bruteforce, sz, 'filled'); hold on;
scatter(idx, RMSE_mlesac, sz, 'filled');
scatter(idx, RMSE_odr_mlesac, sz, 'filled');
scatter(idx, RMSE_odr_bruteforce, sz, 'filled');
hdl = legend('brute-force','MLESAC', ...
    'ODR - MLESAC seed','ODR - brute-force seed');
set(hdl,'Interpreter','latex','Location','best')




