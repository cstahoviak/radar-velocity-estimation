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
%   a. Define a look-up table of valid binned angular locations
%   b. map the truth angular values to a binned angular value
%   c. update value of sigma_theta with average angular resolution

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 2;             % problem uniquely-determined for 2 targets
maxDistance = 0.1;          % only roughly tuned at this point
conditionNum_thres = 100;

%% Define measurement noise parameters

sigma_vr    = 0.04;     % [m/s]
sigma_theta = 2;        % [deg] NOTE: need to refine this value

%% Generate Simulated Radar Measurements

% number of simulated targets
Ntargets = 50;

% hard-coded values
velocity = [0.25, 0.75]'
%     radar_angle = [-4, 4, -30, 30, -55, 55]';
%     Ntargets = size(radar_angle,1);

% simulated target angles
min_angle = -70;    % [deg]
max_angle = 70;     % [deg]
radar_angle = (max_angle-min_angle).*rand(Ntargets,1) + min_angle;

% simulated 'true' platform velocity
min_vel = 0;        % [m/s]
max_vel = 2.5;      % [m/s]
%     velocity = (max_vel-min_vel).*rand(2,1) + min_vel

% define AGWN vector
eps = normrnd(0,sigma_vr,[Ntargets,1]);
delta = normrnd(0,deg2rad(sigma_theta),[Ntargets,1]);

% define error variance ratio, d
d = ones(Ntargets,1)*(sigma_vr/sigma_theta);

% create noisy simulated radar doppler measurements
radar_doppler =  simulateRadarDoppler2D(velocity, ...
    deg2rad(radar_angle), eps, delta);

radar_data = [(1:Ntargets)', radar_angle, radar_doppler]

%% Implement Estimation Schemes

% get 'brute force' estimate of forward/lateral body-frame vel.
[ model, vhat_all ] = getBruteForceEstimate( radar_doppler', ...
    deg2rad(radar_angle)', conditionNum_thres);
fprintf('Brute-Force  Velocity Profile Estimation\n');
disp(model)

% get MLESAC (M-estimator RANSAC) model and inlier set
[ model_mlesac, inlier_idx ] = MLESAC( radar_doppler', ...
    deg2rad(radar_angle'), sampleSize, maxDistance, conditionNum_thres );
fprintf('MLESAC Velocity Profile Estimation\n');
disp(model_mlesac)
fprintf('MLESAC Number of Inliers\n');
disp(sum(inlier_idx));

% get Orthogonal Distance Regression (ODR) estimate
weights = (1/sigma_vr)*ones(Ntargets,1);
[ model_odr, beta ] = ODR( deg2rad(radar_angle)', radar_doppler', ...
    d, model_mlesac, delta, weights )

%% Plot Results

figure(1)
plot(beta(1,:), 'b'); hold on;
plot(beta(2,:), 'r');
plot([0, length(beta)], [velocity(1), velocity(1)], 'b--')
plot([0, length(beta)], [velocity(2), velocity(2)], 'r--')
xlim([0, length(beta)]);
xlabel('iteration index','Interpreter','latex')
ylabel('velocity [m/s]','Interpreter','latex')
title({'Othrogonal Distance Regression (ODR)', 'Velocity Estimate'},...
    'Interpreter','latex')

figure(2)
quiver(0,0,velocity(1),velocity(2),'LineWidth',2); hold on;
quiver(0,0,model(1),model(2));
quiver(0,0,model_mlesac(1),model_mlesac(2));
quiver(0,0,model_odr(1),model_odr(2));
hdl = legend('truth','brute-force','MLESAC','ODR');
set(hdl,'Interpreter','latex','Location','best')
xlabel('$v_x$ [m/s]','Interpreter','latex')
ylabel('$v_y$ [m/s]','Interpreter','latex')
title('Velocity Estimate Comparison','Interpreter','latex')
% xlim([0 1]); ylim([0 1]);

