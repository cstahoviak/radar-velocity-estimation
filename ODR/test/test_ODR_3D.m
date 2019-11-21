%% Header

%%% Filename:   test_ODR_3D.m
%%% Author:     Carl Stahoviak
%%% Created:    05/15/2019  

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

% scaling factor for step s
s = 10*ones(sampleSize,1);

converge_thres = 0.0005;
max_iter = 50;
get_covar = true;

%% Define measurement parameters

% generate simulated pointcloud
type = 'points';

% number of simulated targets
Ninliers = 130;
Noutliers = 35;

%% Generate Simulated Radar Measurements

% hard-coded values
% velocity = [1.2, 0.75]'

% simulated 'true' platform velocity
min_vel = -2.5;        % [m/s]
max_vel = 2.5;      % [m/s]
% velocity = round((max_vel-min_vel).*rand(3,1) + min_vel,2);
velocity = [-1.60, -2.10, 0.90]';

% create noisy simulated radar measurements
[~, ~, ~, inlier_doppler, inlier_azimuth, inlier_elevation] = ...
    getRadarMeasurements_3D( Ninliers, velocity, radar_angle_bins, ...
    sigma_vr, sigma, type );

%% Generate Outlier Data

% create noisy simulated radar measurements
sigma_vr_outlier = 1.5;     % [m/s]
[~, ~, ~, outlier_doppler, outlier_azimuth, outlier_elevation] = ...
    getRadarMeasurements_3D( Noutliers, velocity, radar_angle_bins, ...
    sigma_vr_outlier, sigma, type );

% combine inlier and outlier data sets
Ntargets = Ninliers + Noutliers;
radar_doppler = [inlier_doppler; outlier_doppler];
radar_azimuth = [inlier_azimuth; outlier_azimuth];
radar_elevation = [inlier_elevation; outlier_elevation];

radar_data = [(1:Ntargets)', radar_doppler, radar_azimuth, radar_elevation];

%% Implement Estimation Schemes

% get 'brute force' estimate of forward/lateral body-frame vel.
% tic
% [ model_bruteforce, vhat_all ] = getBruteForceEstimate3D( radar_doppler', ...
%     radar_azimuth', radar_elevation');
% time_bruteforce = toc;

% get Matlab MLESAC (Max. Likelihood RANSAC) model and inlier set
tic
[ model_mlesac, inlier_idx ] = MLESAC_3D( radar_doppler, ...
    radar_azimuth, radar_elevation, sampleSize, maxDistance );
time_mlesac = toc;
Ninliers = sum(inlier_idx);     % probably shouldn't redefine this var

% get doppler_mlesac estimate
data = [radar_doppler, radar_azimuth, radar_elevation];
tic
[ model_mlesac2, inlier_idx2, scores ] = doppler_mlesac( data, n, p, t, ...
    mlesac_max_iter, sigma_vr);
time_mlesac2 = toc;
Ninliers2 = sum(inlier_idx2);

% create data for ODR regression 
weights = (1/sigma_vr)*ones(Ninliers,1);
% weights = (1/sigma_vr)*randi([0,100],Ninliers,1);
data = [radar_doppler(inlier_idx), radar_azimuth(inlier_idx), ...
    radar_elevation(inlier_idx)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TESTING

% tic
% delta_theta = normrnd(0,sigma(1),[1,Ninliers]); 
% delta_phi = normrnd(0,sigma(2),[1,Ninliers]);
% delta0 = [delta_theta; delta_phi];
% delta0 = delta0(:);
% [ model_odr5_t, ~, ~, iter5_t ] = ODR_v5_test( data, d, model_mlesac, ...
%     sigma, weights, s, converge_thres, max_iter, get_covar, delta0 );
% time_odr5_t = toc;
% 
% % get 'old' ODR_3D estimate as comparison
% tic
% [ model_odr_3d, ~, ~, iter_3d ] = ODR_3D( radar_doppler(inlier_idx)', ...
%     radar_azimuth(inlier_idx)', radar_elevation(inlier_idx)', d, model_mlesac, ...
%     [delta_theta'; delta_phi'], weights, converge_thres );
% time_odr_3d = toc;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get 3D Orthogonal Distance Regression (ODR_v1) estimate
tic
[ model_odr1, ~, ~, iter1 ] = ODR_v1( data, d, model_mlesac, ...
    sigma, weights, converge_thres, max_iter, false );
time_odr1 = toc;

% get Orthogonal Distance Regression (ODR_v2) estimate
tic
[ model_odr2, ~, ~, iter2 ] = ODR_v2( data, d, model_mlesac, ...
    sigma, weights, converge_thres, max_iter, false );
time_odr2 = toc;

% get Orthogonal Distance Regression (ODR_v3) estimate
tic
[ model_odr3, ~, ~, iter3 ] = ODR_v3( data, d, model_mlesac, ...
    sigma, weights, converge_thres, max_iter, get_covar );
time_odr3 = toc;

% get 3D Orthogonal Distance Regression (ODR_v4) estimate
tic
[ model_odr4, ~, ~, iter4 ] = ODR_v4( data, d, model_mlesac, ...
    sigma, weights, s, converge_thres, max_iter, get_covar );
time_odr4 = toc;

% get 3D Orthogonal Distance Regression (ODR_v5) estimate
tic
[ model_odr5, beta5, cov, iter5 ] = ODR_v5( data, d, model_mlesac, ...
    sigma, weights, s, converge_thres, max_iter, get_covar );
time_odr5 = toc;

% get 3D Orthogonal Distance Regression (ODR_v5) estimate
% tic
% [ model_odr6, beta6, ~, iter6 ] = ODR_v6( data, d, model_mlesac, ...
%     sigma, weights, ones(3,1), 0.5, 100, get_covar );
% time_odr6 = toc;

% get LSQNONLIN (OLS) solution

f = @(model) doppler_residual( model, data );
x0 = ones(size(velocity,1),1);
tic
model_lsqnonlin = lsqnonlin(f,x0,[],[],opts);
time_lsqnonlin = toc;

% get NLINFIT (OLS) solution
% NOTE NLINFIT() produces the EXACT same result as LSQNONLIN()
pred = [radar_azimuth(inlier_idx), radar_elevation(inlier_idx)];
resp = radar_doppler(inlier_idx);
g = @(model, X) simulateRadarDoppler3D( model, X(:,1), X(:,2), ...
    zeros(Ninliers,1), zeros(2*Ninliers,1));
tic
model_nlinfit = nlinfit(pred,resp,g,x0);
time_nlinfit = toc;

% RMSE_bruteforce     = sqrt(mean((velocity - model_bruteforce).^2))
RMSE_mlesac    = sqrt(mean((velocity - model_mlesac).^2));
RMSE_mlesac2   = sqrt(mean((velocity - model_mlesac2).^2));
RMSE_lsqnonlin = sqrt(mean((velocity - model_lsqnonlin).^2));
RMSE_nlinfit   = sqrt(mean((velocity - model_nlinfit).^2));
RMSE_odr1      = sqrt(mean((velocity - model_odr1).^2));
RMSE_odr2      = sqrt(mean((velocity - model_odr2).^2));
RMSE_odr3      = sqrt(mean((velocity - model_odr3).^2));
RMSE_odr4      = sqrt(mean((velocity - model_odr4).^2));
RMSE_odr5      = sqrt(mean((velocity - model_odr5).^2));
% RMSE_odr6      = sqrt(mean((velocity - model_odr6).^2));

% RMSE_odr5_t     = sqrt(mean((velocity - model_odr5_t).^2));
% RMSE_odr_3d     = sqrt(mean((velocity - model_odr_3d).^2));

%% Algorithm Evaluation

fprintf('Algorithm Evaluation - Parameter Estimate\n');
fprintf('Truth\t\tMatlab MLESAC\tLSQNONLIN (OLS)\t ODR_v5\n');
fprintf('%.4f\t\t%.4f\t\t%.4f\t\t %.4f\n', velocity(1), model_mlesac(1), ...
    model_lsqnonlin(1), model_odr5(1));
fprintf('%.4f\t\t%.4f\t\t%.4f\t\t %.4f\n', velocity(2), model_mlesac(2), ...
    model_lsqnonlin(2), model_odr5(2));
fprintf('%.4f\t\t%.4f\t\t%.4f\t\t %.4f\n', velocity(3), model_mlesac(3), ...
    model_lsqnonlin(3), model_odr5(3));

fprintf('\nAlgorithm Evaluation - RMSE [m/s]\n');
fprintf('Matlab MLESAC\t= %.4f\n', RMSE_mlesac);
fprintf('Doppler MLESAC\t= %.4f\n', RMSE_mlesac2);
fprintf('LSQNONLIN (OLS)\t= %.4f\n', RMSE_lsqnonlin);
fprintf('NLINFIT (OLS)\t= %.4f\n', RMSE_nlinfit);
fprintf('ODR_v1\t\t= %.4f\n', RMSE_odr1);
fprintf('ODR_v2\t\t= %.4f\n', RMSE_odr2);
fprintf('ODR_v3\t\t= %.4f\n', RMSE_odr3);
fprintf('ODR_v4\t\t= %.4f\n', RMSE_odr4);
fprintf('ODR_v5\t\t= %.4f\n', RMSE_odr5);
% fprintf('ODR_v6\t\t= %.4f\n', RMSE_odr6);
% fprintf('ODR_v5_test\t= %.4f\n', RMSE_odr5_t);
% fprintf('ODR_3D\t\t= %.4f\n', RMSE_odr_3d);

fprintf('\nAlgorithm Evaluation - Execution Time [milliseconds]\n');
fprintf('Matlab MLESAC\t= %.4f\n', 1e3*time_mlesac);
fprintf('Doppler MLESAC\t= %.4f\n', 1e3*time_mlesac2);
fprintf('LSQNONLIN (OLS)\t= %.4f\n', 1e3*time_lsqnonlin);
fprintf('NLINFIT (OLS)\t= %.4f\n', 1e3*time_nlinfit);
fprintf('ODR_v1\t\t= %.4f\n', 1e3*time_odr1);
fprintf('ODR_v2\t\t= %.4f\n', 1e3*time_odr2);
fprintf('ODR_v3\t\t= %.4f\n', 1e3*time_odr3);
fprintf('ODR_v4\t\t= %.4f\n', 1e3*time_odr4);
fprintf('ODR_v5\t\t= %.4f\n', 1e3*time_odr5);
% fprintf('ODR_v6\t\t= %.4f\n', 1e3*time_odr6);
% fprintf('ODR_v5_test\t= %.4f\n', 1e3*time_odr5_t);
% fprintf('ODR_3D\t\t= %.4f\n', 1e3*time_odr_3d);

fprintf('\nAlgorithm Evaluation - Misc.\n');
fprintf('Matlab MLESAC Inliers\t= %d\n', Ninliers);
fprintf('Doppler MLESAC Inliers\t= %d\n', Ninliers2);
fprintf('\t\t ODR_v1\t ODR_v2\t ODR_v3\t ODR_v4\t ODR_v5\t ODR_v6\n');
fprintf('ODR Iterations\t %d\t %d\t %d\t %d\t %d\n', iter1, iter2, ...
    iter3, iter4, iter5);

% fprintf('\t\t ODR_v1\t ODR_v2\t ODR_v3\t ODR_v4\t ODR_v5\t ODR_v5t\t ODR_3D\n');
% fprintf('ODR Iterations\t %d\t %d\t %d\t %d\t %d\t %d\t\t %d\n', iter1, iter2, ...
%     iter3, iter4, iter5, iter5_t, iter_3d);

%% Plot Results

% plot results from specific version of ODR
model_odr = model_odr5;
beta      = beta5;

load('colors.mat')

figure(1)
plot(0:1:(size(beta,2)-1), beta(1,:), 'b'); hold on;
plot(0:1:(size(beta,2)-1), beta(2,:), 'r');
plot(0:1:(size(beta,2)-1), beta(3,:), 'm');
plot([0, size(beta,2)-1], [velocity(1), velocity(1)], 'b--')
plot([0, size(beta,2)-1], [velocity(2), velocity(2)], 'r--')
plot([0, size(beta,2)-1], [velocity(3), velocity(3)], 'm--')
xlim([0, size(beta,2)-1]);
xticks(0:1:size(beta,2));
xlabel('iteration index','Interpreter','latex')
ylabel('velocity [m/s]','Interpreter','latex')
title({'Ego-Velocity Estimate','Othrogonal Distance Regression (ODR)'}, ...
    'Interpreter','latex')

figure(2)
quiver3(0,0,0,velocity(1),velocity(2),velocity(3),'--'); hold on;
quiver3(0,0,0,model_mlesac(1),model_mlesac(2),model_mlesac(3));
quiver3(0,0,0,model_lsqnonlin(1),model_lsqnonlin(2),model_lsqnonlin(3));
quiver3(0,0,0,model_odr(1),model_odr(2),model_odr(3));
hdl = legend('truth','Matlab MLESAC','LSQNONLIN (OLS)','ODR');
set(hdl,'Interpreter','latex','Location','best')
xlabel('$v_x$ [m/s]','Interpreter','latex')
ylabel('$v_y$ [m/s]','Interpreter','latex')
ylabel('$v_z$ [m/s]','Interpreter','latex')
title('Ego-Velocity Estimate Comparison','Interpreter','latex')
% xlim([0 1]); ylim([0 1]);

%% Plot velocity Profile

azimuth = linspace(-pi/2,pi/2,100)';
elevation = linspace(-pi/2,pi/2,100)';
N = size(azimuth,1);

% get true velocity profile
profile_truth = simulateRadarDoppler3D(velocity, azimuth, elevation, ...
    zeros(N,1), zeros(2*N,1));

% get brute-force velocity profile
% profile_bruteforce = simulateRadarDoppler3D(model_bruteforce, azimuth, ...
%     elevation, zeros(N,1), zeros(2*N,1));

% get MLESAC velocity profile
profile_mlesac = simulateRadarDoppler3D(model_mlesac, azimuth, ...
    elevation, zeros(N,1), zeros(2*N,1));

% get LSQNONLIN (OLS) velocity profile
profile_ols = simulateRadarDoppler3D(model_lsqnonlin, azimuth, elevation, ...
    zeros(N,1), zeros(2*N,1));

% get ODR velocity profile
profile_odr = simulateRadarDoppler3D(model_odr, azimuth, elevation, ...
    zeros(N,1), zeros(2*N,1));

figure(4)
plot3(azimuth,elevation,profile_truth,'Color',colors(1,:)); hold on
plot3(azimuth,elevation,profile_ols,'--','Color',colors(2,:));
plot3(azimuth,elevation,profile_odr,'--','Color',colors(3,:))
scatter3(radar_azimuth(inlier_idx), radar_elevation(inlier_idx), ...
    radar_doppler(inlier_idx),15,[0.75,0,0.75],'filled')
scatter3(radar_azimuth(~inlier_idx), radar_elevation(~inlier_idx), ...
    radar_doppler(~inlier_idx),30,'kx')
% scatter(outlier_azimuth, outlier_doppler,10,'kx')
xlim([-pi/2, pi/2]);
xlabel('azimuth, $\theta$ [rad]','Interpreter','latex')
ylabel('elevation, $\phi$ [rad]','Interpreter','latex')
zlabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile','Interpreter','latex')
% hdl = legend('true velocity profile','MLESAC velocity profile', ...
%     'MLESAC inliers','MLESAC outliers');
hdl = legend('true velocity profile','OLS velocity profile', ...
    'ODR velocity profile','MLESAC inliers','MLESAC outliers');
set(hdl,'Interpreter','latex')

figure(5)
plot3(azimuth,elevation,profile_truth,'Color',colors(1,:)); hold on
plot3(azimuth,elevation,profile_ols,'--','Color',colors(2,:));
plot3(azimuth,elevation,profile_odr,'--','Color',colors(3,:))
scatter3(radar_azimuth(inlier_idx), radar_elevation(inlier_idx), ...
    radar_doppler(inlier_idx),15,[0.75,0,0.75],'filled')
scatter3(radar_azimuth(~inlier_idx), radar_elevation(~inlier_idx), ...
    radar_doppler(~inlier_idx),30,'kx')
% scatter(outlier_azimuth, outlier_doppler,10,'kx')
xlim([-pi/2 - deg2rad(10), pi/2 + deg2rad(10)]);
ylim([-pi/2 - deg2rad(10), pi/2 + deg2rad(10)]);

for i=1:N
    az = ones(N,1)*azimuth(i);
    elev = ones(N,1)*elevation(i);
    
    doppler_true = simulateRadarDoppler3D(velocity, az, elevation, ...
        zeros(N,1), zeros(2*N,1));
    doppler_ols = simulateRadarDoppler3D(model_lsqnonlin, az, elevation, ...
        zeros(N,1), zeros(2*N,1));
    doppler_odr = simulateRadarDoppler3D(model_odr, az, elevation, ...
        zeros(N,1), zeros(2*N,1));
    
    plot3(az, elevation, doppler_true,'Color',colors(1,:));
    plot3(az, elevation, doppler_ols,'Color',colors(2,:));
    plot3(az, elevation, doppler_odr,'Color',colors(3,:));
    
%     plot3(azimuth, elev, doppler_true,'Color',colors(1,:));
%     plot3(azimuth, elev, doppler_ols,'Color',colors(2,:));
%     plot3(azimuth, elev, doppler_odr,'Color',colors(3,:));
    
end

xlabel('azimuth, $\theta$ [rad]','Interpreter','latex')
ylabel('elevation, $\phi$ [rad]','Interpreter','latex')
zlabel('radial velocity, $v_r$ [m/s]','Interpreter','latex')
title('Cosine Velocity Profile','Interpreter','latex')
hdl = legend('true velocity profile','OLS velocity profile', ...
    'ODR velocity profile','MLESAC inliers','MLESAC outliers');
set(hdl,'Interpreter','latex')

%% Plot doppler MLESAC Convergence Indicators

figure(6)
plot(scores)
xlim([0,length(scores)])
xlabel('iteration','Interpreter','latex')
ylabel('data log-likelihood','Interpreter','latex')
title('MLESAC data log-likelihood','Interpreter','latex')

