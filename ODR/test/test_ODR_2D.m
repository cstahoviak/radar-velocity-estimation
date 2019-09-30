%% Header

%%% Filename:   test_ODR_2D.m
%%% Author:     Carl Stahoviak
%%% Created:    03/04/2019  

clear;
clc;
close all;

format compact

opts = optimset('display','off');   % for LSQNONLIN

%%% TODO:
% 1. Implement angle binning algorithm
%   a. Define a look-up table of valid binned angular locations - DONE
%   b. map the truth angular values to a binned angular value - DONE
%   c. update value of sigma_theta with average angular resolution - DONE

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 2;             % problem uniquely-determined for 2 targets
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
Ninliers = 70;
Noutliers = 35;

%% Generate Simulated Radar Measurements

% hard-coded values
% velocity = [1.2, 0.75]'

% simulated 'true' platform velocity
min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]
velocity = (max_vel-min_vel).*rand(2,1) + min_vel;

% create noisy simulated radar measurements
[~, ~, inlier_doppler, inlier_azimuth ] = getRadarMeasurements( ...
    Ninliers, velocity, radar_angle_bins, sigma );

%% Generate Outlier Data

% create noisy simulated radar measurements
sigma_vr_outlier = 1.5;     % [m/s]
sigma_outlier = [sigma_vr_outlier; sigma_theta];
[~, ~, outlier_doppler, outlier_azimuth ] = getRadarMeasurements( ...
    Noutliers, velocity, radar_angle_bins, sigma_outlier );

% combine inlier and outlier data sets
Ntargets = Ninliers + Noutliers;
radar_doppler = [inlier_doppler; outlier_doppler];
radar_azimuth = [inlier_azimuth; outlier_azimuth];

%% Implement Estimation Schemes

% get 'brute force' estimate of forward/lateral body-frame vel.
% tic
% [ model_bruteforce, vhat_all ] = getBruteForceEstimate( radar_doppler', ...
%     radar_azimuth', conditionNum_thres);
% time_bruteforce = toc;

% get MLESAC (Max. Likelihood RANSAC) model and inlier set
tic
[ model_mlesac, inlier_idx ] = MLESAC( radar_doppler', ...
    radar_azimuth', sampleSize, maxDistance );
time_mlesac = toc;
Ninliers = sum(inlier_idx);

% get Orthogonal Distance Regression (ODR v1) estimate - MLESAC seed
tic
weights = (1/sigma_vr)*ones(Ninliers,1);
data = [radar_doppler(inlier_idx), radar_azimuth(inlier_idx), zeros(Ninliers,1)];
[ model_odr1, beta1, cov1, odr_iter1 ] = ODR_v1( data, d, model_mlesac, ...
    sigma(2), weights, converge_thres, max_iter, get_covar );
time_odr1 = toc;

% get Orthogonal Distance Regression (ODR v2) estimate - MLESAC seed
tic
[ model_odr2, beta2, cov2, odr_iter2 ] = ODR_v2( data, d, model_mlesac, ...
    sigma(2), weights, converge_thres, max_iter, get_covar );
time_odr2 = toc;

% get doppler_mlesac estimate
% data = [radar_doppler, radar_azimuth];
% tic
% [ model_mlesac2, inlier_idx2, scores ] = mlesac( data, n, p, t, sigma_vr);
% time_mlesac2 = toc;
% Ninliers2 = sum(inlier_idx2);

% get LSQNONLIN (OLS) solution
f = @(model) doppler_residual( model, data );
x0 = ones(size(velocity,1),1);
tic
model_lsqnonlin = lsqnonlin(f,x0,[],[],opts);
time_lsqnonlin = toc;

% get NLINFIT (OLS) solution
% NOTE NLINFIT() produces the EXACT same result as LSQNONLIN()
pred = radar_azimuth(inlier_idx);
resp = radar_doppler(inlier_idx);
g = @(model, X) simulateRadarDoppler2D( model, X, ...
zeros(Ninliers,1), zeros(Ninliers,1));
tic
model_nlinfit = nlinfit(pred,resp,g,x0);
time_nlinfit = toc;

% RMSE_bruteforce     = sqrt(mean((velocity - model_bruteforce).^2))
RMSE_mlesac    = sqrt(mean((velocity - model_mlesac).^2));
RMSE_lsqnonlin = sqrt(mean((velocity - model_lsqnonlin).^2));
RMSE_nlinfit   = sqrt(mean((velocity - model_nlinfit).^2));
RMSE_odr1      = sqrt(mean((velocity - model_odr1).^2));
RMSE_odr2      = sqrt(mean((velocity - model_odr2).^2));

%% Algorithm Evaluation

fprintf('Algorithm Evaluation - Parameter Estimate\n\n');
fprintf('Truth\t\tMatlab MLESAC\tLSQNONLIN (OLS)\t ODR_v1 \tODR_v2\n');
fprintf('%.4f\t\t%.4f\t\t%.4f\t\t %.4f \t%.4f\n', velocity(1), model_mlesac(1), ...
    model_lsqnonlin(1), model_odr1(1), model_odr2(1));
fprintf('%.4f\t\t%.4f\t\t%.4f\t\t %.4f \t%.4f\n', velocity(2), model_mlesac(2), ...
    model_lsqnonlin(2), model_odr1(2), model_odr2(2));

fprintf('\nAlgorithm Evaluation - RMSE\n');
fprintf('Matlab MLESAC\t= %.4f\n', RMSE_mlesac);
fprintf('LSQNONLIN (OLS)\t= %.4f\n', RMSE_lsqnonlin);
fprintf('NLINFIT (OLS)\t= %.4f\n', RMSE_nlinfit);
fprintf('ODR_v1\t\t= %.4f\n', RMSE_odr1);
fprintf('ODR_v2\t\t= %.4f\n', RMSE_odr2);

fprintf('\nAlgorithm Evaluation - Execution Time\n');
fprintf('Matlab MLESAC\t= %.4f\n', 1e3*time_mlesac);
fprintf('LSQNONLIN (OLS)\t= %.4f\n', 1e3*time_lsqnonlin);
fprintf('NLINFIT (OLS)\t= %.4f\n', 1e3*time_nlinfit);
fprintf('ODR_v1\t\t= %.4f\n', 1e3*time_odr1);
fprintf('ODR_v2\t\t= %.4f\n', 1e3*time_odr2);

fprintf('\nAlgorithm Evaluation - Misc.\n');
fprintf('Matlab MLESAC Inliers\t= %d\n', Ninliers);
fprintf('ODR_v1 Iterations\t= %d\n', odr_iter1);
fprintf('ODR_v2 Iterations\t= %d\n', odr_iter2);

return;

%% Plot Results

load('colors.mat')

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
quiver(0,0,velocity(1),velocity(2),'--'); hold on;
quiver(0,0,model_mlesac(1),model_mlesac(2));
quiver(0,0,model_lsqnonlin(1),model_lsqnonlin(2));
quiver(0,0,model_odr1(1),model_odr1(2));
hdl = legend('truth','Matlab MLESAC','LSQNONLIN (OLS)','ODR');
set(hdl,'Interpreter','latex','Location','best')
xlabel('$v_x$ [m/s]','Interpreter','latex')
ylabel('$v_y$ [m/s]','Interpreter','latex')
title('Ego-Velocity Estimate Comparison','Interpreter','latex')
% xlim([0 1]); ylim([0 1]);

%% Plot velocity Profile

azimuth = linspace(-pi/2,pi/2,1000)';
N = size(azimuth,1);

% get true velocity profile
profile_truth = simulateRadarDoppler2D(velocity, azimuth, ...
    zeros(N,1), zeros(N,1));

% get brute-force velocity profile
% profile_bruteforce = simulateRadarDoppler2D(model_bruteforce, azimuth, ...
%     zeros(N,1), zeros(N,1));

% get MLESAC velocity profile
profile_mlesac = simulateRadarDoppler2D(model_mlesac, azimuth, ...
    zeros(N,1), zeros(N,1));

% get LSQNONLIN (OLS) velocity profile
profile_ols = simulateRadarDoppler2D(model_lsqnonlin, azimuth, ...
    zeros(N,1), zeros(N,1));

% get ODR velocity profile
profile_odr = simulateRadarDoppler2D(model_odr1, azimuth, ...
    zeros(N,1), zeros(N,1));

figure(4)
plot(azimuth,profile_truth,'Color',colors(1,:)); hold on
plot(azimuth,profile_ols,'--','Color',colors(2,:));
plot(azimuth,profile_odr,'--','Color',colors(3,:))
scatter(radar_azimuth(inlier_idx), radar_doppler(inlier_idx), ...
    15,[0.75,0,0.75],'filled')
scatter(radar_azimuth(~inlier_idx), radar_doppler(~inlier_idx),30,'kx')
% scatter(outlier_azimuth, outlier_doppler,10,'kx')
xlim([-pi/2, pi/2]);
xlabel('azimuth, $\theta$ [rad]','Interpreter','latex')
ylabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile - MLESAC','Interpreter','latex')
% hdl = legend('true velocity profile','MLESAC velocity profile', ...
%     'MLESAC inliers','MLESAC outliers');
hdl = legend('true velocity profile','OLS velocity profile', ...
    'ODR velocity profile','MLESAC inliers','MLESAC outliers');
set(hdl,'Interpreter','latex')

return;

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



