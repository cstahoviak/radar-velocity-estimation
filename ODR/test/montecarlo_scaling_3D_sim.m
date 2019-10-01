%% Header

%%% Filename:   montecarlo_scaling_3D.m
%%% Author:     Carl Stahoviak
%%% Created:    10/01/2019  

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

%% Define ODR Parameters

load('1642_azimuth_bins.mat')

sigma_vr = 0.044;                   % [m/s]
sigma_phi = sigma_theta;            % [rad]
sigma = [sigma_theta; sigma_phi];

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

converge_thres = 0.0005;
max_iter = 50;
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

n = 50;             % number of scaling values to test
mc_iter = 150;

scaling = [.1:.1:.9, linspace(1,50,n)];
M = length(scaling);

rmse_stats = NaN*ones(4,4,M);
time_stats = NaN*ones(4,4,M);
iter_stats = NaN*ones(2,5,M);

for m=1:M
    
    fprintf('Scale Factor Iteration: %d\n', m);
    
    % initialize
    rmse = NaN*ones(mc_iter,4);     % [mlesac, lsqnonlin, odr_v4, odr_v5]
    time = NaN*ones(mc_iter,4);     % [mlesac, lsqnonlin, odr_v4, odr_v5]
    inliers = NaN*ones(mc_iter,1);
    odr_iter = NaN*ones(mc_iter,2);
    
    s = scaling(m);

    for i=1:mc_iter

        fprintf('\tMC Iteration: %d\n', i);

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
        [ model_mlesac, inlier_idx ] = MLESAC_3D( radar_doppler', ...
            radar_azimuth', radar_elevation', sampleSize, maxDistance );
        time(i,1) = toc;
        inliers(i) = sum(inlier_idx);

        % get Orthogonal Distance Regression (ODR_v4) estimate
        weights = (1/sigma_vr)*ones(inliers(i),1);
        data = [radar_doppler(inlier_idx), radar_azimuth(inlier_idx), ...
            radar_elevation(inlier_idx)];
        tic
        [ model_odr4, ~, ~, iter4 ] = ODR_v4( data, d, model_mlesac, ...
            sigma, weights, s, converge_thres, max_iter, get_covar );
        time(i,3) = toc;

        % get Orthogonal Distance Regression (ODR_v5) estimate
        tic
        [ model_odr5, ~, ~, iter5 ] = ODR_v5( data, d, model_mlesac, ...
            sigma, weights, s, converge_thres, max_iter, get_covar );
        time(i,4) = toc;

        % get LSQNONLIN (OLS) solution
        f = @(model) doppler_residual( model, data );
        x0 = ones(size(velocity,1),1);
        tic
        model_lsqnonlin = lsqnonlin(f,x0,[],[],opts);
        time(i,2) = toc;

        % compute RMSE statistics
        rmse(i,1) = sqrt(mean((velocity - model_mlesac).^2));
        rmse(i,2) = sqrt(mean((velocity - model_lsqnonlin).^2));
        rmse(i,3) = sqrt(mean((velocity - model_odr4).^2));
        rmse(i,4) = sqrt(mean((velocity - model_odr5).^2));

        odr_iter(i,:) = [iter4, iter5];
    end
    
    % convert time to milliseconds
    time = 1e3*time;
    
    rmse_stats(:,:,m) = [mean(rmse,1)', std(rmse,1)', min(rmse)', max(rmse)'];
    time_stats(:,:,m) = [mean(time,1)', std(time,1)', min(time)', max(time)'];
    iter_stats(:,:,m) = [mean(odr_iter,1)', std(odr_iter,1)', min(odr_iter)', ...
        max(odr_iter)', mode(odr_iter,1)'];
    
end

%% Plot Data

% plot RMSE statistics - no error bars
figure(1)
plot(scaling,squeeze(rmse_stats(1,1,:)),'-d'); hold on
plot(scaling,squeeze(rmse_stats(2,1,:)),'-*');
plot(scaling,squeeze(rmse_stats(3,1,:)),'-s');
plot(scaling,squeeze(rmse_stats(4,1,:)),'-x');

title({'Algorithm Evaluation','Ego-Velocity RMSE'},'Interpreter','latex')
xlabel('step size, $s$','Interpreter','latex')
ylabel('RMSE [m/s]','Interpreter','latex')
hdl = legend('Matlab MLSEAC','LSQNONLIN','ODR v4','ODR v5');
set(hdl,'Interpreter','latex')
xlim([0, scaling(end)]);

% plot RMSE statistics - with error bars
figure(2)
errorbar(scaling,squeeze(rmse_stats(1,1,:)),squeeze(rmse_stats(1,2,:))); hold on
errorbar(scaling,squeeze(rmse_stats(2,1,:)),squeeze(rmse_stats(2,2,:)));
errorbar(scaling,squeeze(rmse_stats(3,1,:)),squeeze(rmse_stats(3,2,:)));
errorbar(scaling,squeeze(rmse_stats(4,1,:)),squeeze(rmse_stats(4,2,:)));

title({'Algorithm Evaluation','Ego-Velocity RMSE'},'Interpreter','latex')
xlabel('step size, $s$','Interpreter','latex')
ylabel('RMSE [m/s]','Interpreter','latex')
hdl = legend('Matlab MLSEAC','LSQNONLIN','ODR v4','ODR v5');
set(hdl,'Interpreter','latex')
xlim([0, scaling(end)]);

% plot execution time statistics - no error bars
figure(3)
errorbar(scaling,squeeze(time_stats(1,1,:)),squeeze(time_stats(1,2,:))); hold on
errorbar(scaling,squeeze(time_stats(2,1,:)),squeeze(time_stats(2,2,:)));
errorbar(scaling,squeeze(time_stats(3,1,:)),squeeze(time_stats(3,2,:)));
errorbar(scaling,squeeze(time_stats(4,1,:)),squeeze(time_stats(4,2,:)));

title({'Algorithm Evaluation','Execution Time'},'Interpreter','latex')
xlabel('step size, $s$','Interpreter','latex')
ylabel('execution time [ms]','Interpreter','latex')
hdl = legend('Matlab MLSEAC','LSQNONLIN','ODR v4','ODR v5');
set(hdl,'Interpreter','latex')
xlim([0, scaling(end)]);

figure(4)
plot(scaling,squeeze(rmse_stats(2,1,:))-squeeze(rmse_stats(3,1,:)),'-d'); hold on
plot(scaling,squeeze(rmse_stats(2,1,:))-squeeze(rmse_stats(4,1,:)),'-d');
plot([0,scaling(end)],[0,0],'--r')

title({'Algorithm Evaluation','Ego-Velocity RMSE -- LSQNONLIN vs. ODR'}, ...
    'Interpreter','latex')
xlabel('step size, $s$','Interpreter','latex')
ylabel('RMSE difference [m/s]','Interpreter','latex')
hdl = legend('ODR v4','ODR v5');
set(hdl,'Interpreter','latex')
xlim([0, scaling(end)]);




