%% Header

%%% Filename:   test_ODR.m
%%% Author:     Carl Stahoviak
%%% Created:    03/04/2019  

clear;
clc;
close all;

format compact

%%% TODO:
% 1. Implement angle binning algorithm
%   a. Define a look-up table of valid binned angular locations - DONE
%   b. map the truth angular values to a binned angular value - DONE
%   c. update value of sigma_theta with average angular resolution - DONE

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 2;             % problem uniquely-determined for 2 targets
maxDistance = 0.15;          % only roughly tuned at this point
% maxDistance = 0.1;          % only roughly tuned at this point
conditionNum_thres = 100;

n = sampleSize;     % minimum number of points needed to fit the model
p = 0.95;           % probability of sampling at least one good inlier set
t = maxDistance;    % threshold to declare inlier/outlier

%% Define measurement parameters

% number of simulated targets
Ninliers = 75;
Noutliers = 10;

sigma_vr = 0.044;     % [m/s]
load('radar_angle_bins.mat')
% sigma_theta = 0.0413;

%% Generate Simulated Radar Measurements

% hard-coded values
% velocity = [1.2, 0.75]'

% simulated 'true' platform velocity
min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]
velocity = (max_vel-min_vel).*rand(2,1) + min_vel

% create noisy simulated radar measurements
[~, ~, inlier_azimuth, inlier_doppler ] = getRadarMeasurements( ...
    Ninliers, velocity, radar_angle_bins, sigma_vr );

%% Generate Outlier Data

% create noisy simulated radar measurements
sigma_vr_outlier = 1.0;     % [m/s]
[~, ~, outlier_azimuth, outlier_doppler ] = getRadarMeasurements( ...
    Noutliers, velocity, radar_angle_bins, sigma_vr_outlier );

% combine inlier and outlier data sets
Ntargets = Ninliers + Noutliers;
radar_doppler = [inlier_doppler; outlier_doppler];
radar_azimuth = [inlier_azimuth; outlier_azimuth];

% radar_data = [(1:Ntargets)', true_angle, radar_azimuth, ...
%     true_doppler, radar_doppler];

%% Implement Estimation Schemes

% get 'brute force' estimate of forward/lateral body-frame vel.
[ model_bruteforce, vhat_all ] = getBruteForceEstimate( radar_doppler', ...
    radar_azimuth', conditionNum_thres);
fprintf('Brute-Force  Velocity Profile Estimation\n');
disp(model_bruteforce)

% get MLESAC (Max. Likelihood RANSAC) model and inlier set
tic
[ model_mlesac, inlier_idx ] = MLESAC( radar_doppler', ...
    radar_azimuth', sampleSize, maxDistance, conditionNum_thres );
toc
Ninliers = sum(inlier_idx);
fprintf('MLESAC Velocity Profile Estimation\n');
disp(model_mlesac)
fprintf('MLESAC Number of Inliers\n');
disp(Ninliers);

% get Orthogonal Distance Regression (ODR) estimate - MLESAC seed
weights = (1/sigma_vr)*ones(Ninliers,1);
delta = normrnd(0,sigma_theta,[Ninliers,1]);
d = ones(Ninliers,1)*(sigma_vr/sigma_theta);    % error variance ratio
[ model_odr, beta ] = ODR( radar_azimuth(inlier_idx)', ...
    radar_doppler(inlier_idx)', d, model_mlesac, delta, weights );
fprintf('ODR Velocity Profile Estimation - MLESAC seed\n');
disp(model_odr)

% get doppler_mlesac estimate
data = [radar_doppler, radar_azimuth];
tic
[ model_mlesac2, inlier_idx2, scores ] = mlesac( data, n, p, t, sigma_vr);
toc
Ninliears2 = sum(inlier_idx2)
fprintf('doppler MLESAC Velocity Profile Estimation\n');
disp(model_mlesac2)
fprintf('doppler MLESAC Number of Inliers\n');
disp(Ninliears2);

% get Orthogonal Distance Regression (ODR) estimate - doppler_mlesac seed
weights = (1/sigma_vr)*ones(Ninliears2,1);
delta = normrnd(0,sigma_theta,[Ninliears2,1]);
d = ones(Ninliears2,1)*(sigma_vr/sigma_theta);    % error variance ratio
[ model_odr2, beta2 ] = ODR( radar_azimuth(inlier_idx2)', ...
    radar_doppler(inlier_idx2)', d, model_mlesac2, delta, weights );
fprintf('ODR Velocity Profile Estimation - doppler_mlesac seed\n');
disp(model_odr2)

RMSE_bruteforce     = sqrt(mean((velocity - model_bruteforce).^2))
RMSE_mlesac         = sqrt(mean((velocity - model_mlesac).^2))
RMSE_mlesac2        = sqrt(mean((velocity - model_mlesac2).^2))
RMSE_odr_mlesac     = sqrt(mean((velocity - model_odr).^2))
RMSE_odr_mlesac2    = sqrt(mean((velocity - model_odr2).^2))

% return;

%% Plot Results

load('colors.mat')

figure(1)
plot(beta(1,:), 'b'); hold on;
plot(beta(2,:), 'r');
plot([1, length(beta)], [velocity(1), velocity(1)], 'b--')
plot([1, length(beta)], [velocity(2), velocity(2)], 'r--')
xlim([1, length(beta)]);
xlabel('iteration index','Interpreter','latex')
ylabel('velocity [m/s]','Interpreter','latex')
title({'Othrogonal Distance Regression (ODR) - MLESAC Seed', ...
    'Velocity Estimate'},'Interpreter','latex')

figure(2)
plot(beta2(1,:), 'b'); hold on;
plot(beta2(2,:), 'r');
plot([1, length(beta2)], [velocity(1), velocity(1)], 'b--')
plot([1, length(beta2)], [velocity(2), velocity(2)], 'r--')
xlim([1, length(beta2)]);
xlabel('iteration index','Interpreter','latex')
ylabel('velocity [m/s]','Interpreter','latex')
title({'Othrogonal Distance Regression (ODR) - doppler MLESAC seed', ...
    'Velocity Estimate'},'Interpreter','latex')

figure(3)
quiver(0,0,velocity(1),velocity(2),'--'); hold on;
quiver(0,0,model_bruteforce(1),model_bruteforce(2));
quiver(0,0,model_mlesac(1),model_mlesac(2));
quiver(0,0,model_mlesac2(1),model_mlesac2(2));
quiver(0,0,model_odr(1),model_odr(2));
quiver(0,0,model_odr2(1),model_odr2(2));
% hdl = legend('truth','brute-force','MLESAC', ...
%     'ODR');
hdl = legend('truth','brute-force','MLESAC', 'doppler MLESAC', ...
    'ODR - MLESAC seed','ODR - doppler MLESAC seed');
set(hdl,'Interpreter','latex','Location','best')
xlabel('$v_x$ [m/s]','Interpreter','latex')
ylabel('$v_y$ [m/s]','Interpreter','latex')
title('Velocity Estimate Comparison','Interpreter','latex')
% xlim([0 1]); ylim([0 1]);

%% Plot velocity Profile

angles = linspace(-pi/2,pi/2,1000)';
Ntargets = size(angles,1);

% get true velocity profile
profile = simulateRadarDoppler2D(velocity, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));

% get brute-force velocity profile
profile_bruteforce = simulateRadarDoppler2D(model_bruteforce, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));

% get MLESAC velocity profile
profile_mlesac = simulateRadarDoppler2D(model_mlesac, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));
profile_mlesac2 = simulateRadarDoppler2D(model_mlesac2, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));

% get ODR velocity profile
profile_odr = simulateRadarDoppler2D(model_odr, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));
profile_odr2 = simulateRadarDoppler2D(model_odr2, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));

figure(4)
plot(angles,profile); hold on
% plot(angles,profile_bruteforce);
plot(angles,profile_mlesac,'--');
plot(angles,profile_odr,'--')
scatter(radar_azimuth(inlier_idx), radar_doppler(inlier_idx))
scatter(radar_azimuth(~inlier_idx), radar_doppler(~inlier_idx),25,'kx')
% scatter(outlier_azimuth, outlier_doppler,10,'kx')
xlim([-pi/2, pi/2]);
xlabel('target angle, $\theta$ [rad]','Interpreter','latex')
ylabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile - MLESAC','Interpreter','latex')
% hdl = legend('true velocity profile','MLESAC velocity profile', ...
%     'MLESAC inliers','MLESAC outliers');
hdl = legend('true velocity profile','MLESAC velocity profile', ...
    'ODR velocity profile','MLESAC inliers','MLESAC outliers');
set(hdl,'Interpreter','latex','Location','best')

figure(5)
plot(angles,profile); hold on
plot(angles,profile_mlesac2,'--');
plot(angles,profile_odr2,'--')
scatter(radar_azimuth(inlier_idx2), radar_doppler(inlier_idx2))
scatter(radar_azimuth(~inlier_idx2), radar_doppler(~inlier_idx2),25,'kx')
xlim([-pi/2, pi/2]);
xlabel('target angle, $\theta$ [rad]','Interpreter','latex')
ylabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile - doppler MLESAC','Interpreter','latex')
hdl = legend('velocity profile','MLESAC inliers','MLESAC outliers','true outliers');
set(hdl,'Interpreter','latex')

%% Plot MLESAC Convergence Indicators

figure(6)
plot(scores)
xlim([0,length(scores)])
xlabel('iteration','Interpreter','latex')
ylabel('data log-likelihood','Interpreter','latex')
title('MLESAC data log-likelihood','Interpreter','latex')

% figure(7)
% plot(k(:,1)); hold on;
% plot(k(:,2));
% plot(k(:,3));
% xlabel('iteration','Interpreter','latex')
% ylabel('convergence criteria','Interpreter','latex')
% title({'MLESAC Convergence Criteria','$log(1-p)\cdot log(1-w^2)$'}, ...
%     'Interpreter','latex')



