%% Header

%%% Filename:   test_ODR_3D.m
%%% Author:     Carl Stahoviak
%%% Created:    05/15/2019  

clear;
clc;
close all;

format compact

%%% TODO:
% 1. Implement angle binning algorithm
%   a. Define a look-up table of valid binned elevation locations
%   b. map the truth elevation values to a binned angular value
%   c. update value of sigma_phi with average angular resolution

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 3;                 % problem uniquely-determined for 2 targets
maxDistance = 0.15;          % only roughly tuned at this point

n = sampleSize;     % minimum number of points needed to fit the model
p = 0.95;           % probability of sampling at least one good inlier set
t = maxDistance;    % threshold to declare inlier/outlier

%% Define measurement parameters

% generate simulated pointcloud
type = 'points';

% number of simulated targets
Ninliers = 75;
Noutliers = 10;

sigma_vr = 0.044;               % [m/s]
load('radar_angle_bins.mat')
% sigma_theta = 0.0413;           % [rad]
sigma_phi = sigma_theta;        % [rad]

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

%% Generate Simulated Radar Measurements

% hard-coded values
% velocity = [1.2, 0.75]'

% simulated 'true' platform velocity
min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]
velocity = (max_vel-min_vel).*rand(3,1) + min_vel

% create noisy simulated radar measurements
[~, ~, ~, inlier_doppler, inlier_azimuth, inlier_elevation] = ...
    getRadarMeasurements_3D( Ninliers, velocity, radar_angle_bins, sigma_vr, type );

%% Generate Outlier Data

% create noisy simulated radar measurements
sigma_vr_outlier = 1.0;     % [m/s]
[~, ~, ~, outlier_doppler, outlier_azimuth, outlier_elevation] = ...
    getRadarMeasurements_3D( Noutliers, velocity, radar_angle_bins, sigma_vr_outlier, type );

% combine inlier and outlier data sets
Ntargets = Ninliers + Noutliers;
radar_doppler = [inlier_doppler; outlier_doppler];
radar_azimuth = [inlier_azimuth; outlier_azimuth];
radar_elevation = [inlier_elevation; outlier_elevation];

radar_data = [(1:Ntargets)', radar_doppler, radar_azimuth, radar_elevation];

%% Implement Estimation Schemes

% get 'brute force' estimate of forward/lateral body-frame vel.
[ model_bruteforce, vhat_all ] = getBruteForceEstimate3D( radar_doppler', ...
    radar_azimuth', radar_elevation');
fprintf('Brute-Force  Velocity Profile Estimation\n');
disp([velocity, model_bruteforce])

% get MLESAC (Max. Likelihood RANSAC) model and inlier set
[ model_mlesac, inlier_idx ] = MLESAC_3D( radar_doppler', ...
    radar_azimuth', radar_elevation', sampleSize, maxDistance );
Ninliers = sum(inlier_idx);
fprintf('3D MLESAC Velocity Profile Estimation\n');
disp([velocity, model_mlesac])
fprintf('3D MLESAC Number of Inliers\n');
disp(Ninliers);

% get 3D Orthogonal Distance Regression (ODR) estimate - MLESAC seed
weights = (1/sigma_vr)*ones(Ninliers,1);
delta = normrnd(0,sigma_theta,[2*Ninliers,1]);
[ model_odr, beta, cov ] = ODR_3D( radar_doppler(inlier_idx)', ...
    radar_azimuth(inlier_idx)', radar_elevation(inlier_idx)', ...
    d, model_mlesac, delta, weights );
fprintf('ODR Velocity Profile Estimation - MLESAC seed\n');
disp([velocity, model_odr])

RMSE_bruteforce     = sqrt(mean((velocity - model_bruteforce).^2))
RMSE_mlesac_3D      = sqrt(mean((velocity - model_mlesac).^2))
RMSE_odr_3D         = sqrt(mean((velocity - model_odr).^2))

%% Plot Results

load('colors.mat')

figure(1)
plot(beta(1,:), 'b'); hold on;
plot(beta(2,:), 'r');
plot(beta(3,:), 'm');
plot([1, length(beta)], [velocity(1), velocity(1)], 'b--')
plot([1, length(beta)], [velocity(2), velocity(2)], 'r--')
plot([1, length(beta)], [velocity(3), velocity(3)], 'm--')
xlim([1, length(beta)]);
xlabel('iteration index','Interpreter','latex')
ylabel('velocity [m/s]','Interpreter','latex')
title({'Othrogonal Distance Regression (ODR) - MLESAC Seed', ...
    'Velocity Estimate'},'Interpreter','latex')

figure(2)
quiver3(0,0,0,velocity(1),velocity(2),velocity(3),'--'); hold on;
quiver3(0,0,0,model_bruteforce(1),model_bruteforce(2),model_bruteforce(3));
quiver3(0,0,0,model_mlesac(1),model_mlesac(2),model_mlesac(3));
quiver3(0,0,0,model_odr(1),model_odr(2),model_odr(3));
hdl = legend('truth','brute-force','MLESAC', ...
    'ODR');
% hdl = legend('truth','brute-force','MLESAC', 'doppler MLESAC', ...
%     'ODR - MLESAC seed','ODR - doppler MLESAC seed');
set(hdl,'Interpreter','latex','Location','best')
xlabel('$v_x$ [m/s]','Interpreter','latex')
ylabel('$v_y$ [m/s]','Interpreter','latex')
ylabel('$v_z$ [m/s]','Interpreter','latex')
title('Velocity Estimate Comparison','Interpreter','latex')
% xlim([0 1]); ylim([0 1]);

%% Plot velocity Profile

azimuth = linspace(-pi/2,pi/2,1000)';
elevation = linspace(-pi/2,pi/2,1000)';
N = size(azimuth,1);

% get true velocity profile
profile = simulateRadarDoppler3D(velocity, azimuth, elevation, ...
    zeros(N,1), zeros(2*N,1));

% get brute-force velocity profile
profile_bruteforce = simulateRadarDoppler3D(model_bruteforce, azimuth, ...
    elevation, zeros(N,1), zeros(2*N,1));

% get MLESAC velocity profile
profile_mlesac = simulateRadarDoppler3D(model_mlesac, azimuth, ...
    elevation, zeros(N,1), zeros(2*N,1));

% get ODR velocity profile
profile_odr = simulateRadarDoppler3D(model_odr, azimuth, elevation, ...
    zeros(N,1), zeros(2*N,1));

figure(4)
plot3(azimuth,elevation,profile); hold on
% plot(angles,profile_bruteforce);
plot3(azimuth,elevation,profile_mlesac);
plot3(azimuth,elevation,profile_odr)
scatter3(radar_azimuth(inlier_idx), radar_elevation(inlier_idx), ...
    radar_doppler(inlier_idx))
scatter3(radar_azimuth(~inlier_idx), radar_elevation(~inlier_idx), ...
    radar_doppler(~inlier_idx),25,'kx')
% scatter(outlier_azimuth, outlier_doppler,10,'kx')
xlim([-pi/2, pi/2]);
xlabel('azimuth, $\theta$ [rad]','Interpreter','latex')
ylabel('elevation, $\phi$ [rad]','Interpreter','latex')
zlabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile','Interpreter','latex')
% hdl = legend('true velocity profile','MLESAC velocity profile', ...
%     'MLESAC inliers','MLESAC outliers');
hdl = legend('true velocity profile','MLESAC velocity profile', ...
    'ODR velocity profile','MLESAC inliers','MLESAC outliers');
set(hdl,'Interpreter','latex')

% figure(5)
% plot(angles,profile); hold on
% scatter(radar_azimuth(inlier_idx), radar_doppler(inlier_idx))
% scatter(radar_azimuth(~inlier_idx), radar_doppler(~inlier_idx))
% scatter(outlier_azimuth, outlier_doppler,10,'kx')
% xlim([-pi/2, pi/2]);
% xlabel('target angle, $\theta$ [rad]','Interpreter','latex')
% ylabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
% title('Cosine Velocity Profile - MLESAC','Interpreter','latex')
% hdl = legend('velocity profile','MLESAC inliers','MLESAC outliers','true outliers');
% set(hdl,'Interpreter','latex')

return;

%% Plot MLESAC Convergence Indicators

% figure(6)
% plot(scores)
% xlim([0,length(scores)])
% xlabel('iteration','Interpreter','latex')
% ylabel('data log-likelihood','Interpreter','latex')
% title('MLESAC data log-likelihood','Interpreter','latex')

% figure(7)
% plot(k(:,1)); hold on;
% plot(k(:,2));
% plot(k(:,3));
% xlabel('iteration','Interpreter','latex')
% ylabel('convergence criteria','Interpreter','latex')
% title({'MLESAC Convergence Criteria','$log(1-p)\cdot log(1-w^2)$'}, ...
%     'Interpreter','latex')



