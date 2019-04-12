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
maxDistance = 0.1;          % only roughly tuned at this point
conditionNum_thres = 100;

%% Define measurement parameters

% number of simulated targets
Ntargets = 75;

sigma_vr = 0.044;     % [m/s]
load('radar_angle_bins.mat')

% define error variance ratio, d
d = ones(Ntargets,1)*(sigma_vr/sigma_theta);

%% Generate Simulated Radar Measurements

% hard-coded values
velocity = [1.2, 0.75]'

% simulated 'true' platform velocity
min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]
% velocity = (max_vel-min_vel).*rand(2,1) + min_vel

% create noisy simulated radar measurements
[true_angle, true_doppler, radar_angle, radar_doppler ] = ...
    getRadarMeasurements( Ntargets, velocity, radar_angle_bins, sigma_vr );

radar_data = [(1:Ntargets)', true_angle, radar_angle, ...
    true_doppler, radar_doppler];

%% Implement Estimation Schemes

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
fprintf('ODR Velocity Profile Estimation - MLESAC seed\n');
disp(model_odr1)

% get Orthogonal Distance Regression (ODR) estimate - brute-force seed
weights = (1/sigma_vr)*ones(Ntargets,1);
delta = normrnd(0,sigma_theta,[Ntargets,1]);
[ model_odr2, beta2 ] = ODR( radar_angle', radar_doppler', d, ...
    model_bruteforce, delta, weights );
fprintf('ODR Velocity Profile Estimation - brute-force seed\n');
disp(model_odr2)

RMSE_bruteforce     = sqrt(mean((velocity - model_bruteforce).^2))
RMSE_mlesac         = sqrt(mean((velocity - model_mlesac).^2))
RMSE_odr_mlesac     = sqrt(mean((velocity - model_odr1).^2))
RMSE_odr_bruteforce = sqrt(mean((velocity - model_odr2).^2))

%% Plot Results

figure(1)
plot(beta1(1,:), 'b'); hold on;
plot(beta1(2,:), 'r');
plot([1, length(beta1)], [velocity(1), velocity(1)], 'b--')
plot([1, length(beta1)], [velocity(2), velocity(2)], 'r--')
xlim([1, length(beta1)]);
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
title({'Othrogonal Distance Regression (ODR) - Brute-Force Seed', ...
    'Velocity Estimate'},'Interpreter','latex')

figure(3)
quiver(0,0,velocity(1),velocity(2),'LineWidth',2); hold on;
quiver(0,0,model_bruteforce(1),model_bruteforce(2));
quiver(0,0,model_mlesac(1),model_mlesac(2));
quiver(0,0,model_odr1(1),model_odr1(2));
% quiver(0,0,model_odr2(1),model_odr2(2));
hdl = legend('truth','brute-force','MLESAC', ...
    'ODR');
% hdl = legend('truth','brute-force','MLESAC', ...
%     'ODR - MLESAC seed','ODR - brute-force seed');
set(hdl,'Interpreter','latex','Location','best')
xlabel('$v_x$ [m/s]','Interpreter','latex')
ylabel('$v_y$ [m/s]','Interpreter','latex')
title('Velocity Estimate Comparison','Interpreter','latex')
% xlim([0 1]); ylim([0 1]);

%% Plot velocity Profile

angles = linspace(-pi/2,pi/2,1000)';
Ntargets = size(angles,1);

% get true radar doppler measurements
doppler = simulateRadarDoppler2D(velocity, angles, ...
    zeros(Ntargets,1), zeros(Ntargets,1));

figure(4)
plot(angles,doppler)
xlim([-pi/2, pi/2]);
xlabel('target angle, $\theta$ [rad]','Interpreter','latex')
ylabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile','Interpreter','latex')

