%% Header

%%% Filename:   test_Geometry3D.m
%%% Author:     Carl Stahoviak
%%% Created:    04/17/2019  

clc;
clear;
close all;

format compact

%% Define MLESAC parameters

% define MLESAC parameters
sampleSize = 3;                 % problem uniquely-determined for 2 targets
maxDistance = 0.225;          % only roughly tuned at this point

%% Define measurement parameters

sigma_vr = 0.044;               % [m/s]
load('radar_angle_bins.mat')
sigma_theta = 0.0413;           % [rad]
sigma_phi = sigma_theta;        % [rad]

% define ODR error variance ratios
d = [sigma_vr/sigma_theta; sigma_vr/sigma_phi];

%% Main

type = 'points';

% number of simulated targets
Ntargets = 50;

% hard-coded values
% velocity = [1.25, 0, .5]'
% radar_angle = [-4, 4, -30, 30, -55, 55]';
% Ntargets = size(radar_angle,1);

% simulated 'true' platform velocity
min_vel = 0;        % [m/s]
max_vel = 2.5;      % [m/s]
velocity = (max_vel-min_vel).*rand(3,1) + min_vel

if strcmp(type,'angles')
    % simulated target azimuth angles
    min_azimuth = -70;    % [deg]
    max_azimuth = 70;     % [deg]
    radar_azimuth = (max_azimuth - min_azimuth).*rand(Ntargets,1) + min_azimuth;
    radar_azimuth = deg2rad(radar_azimuth);
    
    % simulated target elevation angles
    min_elevation = -70;    % [deg]
    max_elevation = 70;     % [deg]
    radar_elevation = (max_elevation - min_elevation).*rand(Ntargets,1) + min_elevation;
    radar_elevation = deg2rad(radar_elevation);
elseif strcmp(type,'points')
    ptcloud = generatePointcloud3D( Ntargets );
    
    radar_x = ptcloud(:,1);
    radar_y = ptcloud(:,2);
    radar_z = ptcloud(:,3);
%     radar_z = zeros(Ntargets,1);
    
    range = sqrt(radar_x.^2 + radar_y.^2 + radar_z.^2);
    r_xy = sqrt(radar_x.^2 + radar_y.^2);
    
    radar_azimuth = atan(radar_y./radar_x);   % [rad];
    radar_elevation = atan(radar_z./r_xy);   % [rad];
    radar_elevation2 = asin(radar_z./range);
    disp([radar_elevation radar_elevation2])
    
else
    radar_azimuth = zeros(Ntargets,1);
    radar_elevation = zeros(Ntargets,1);
end

% define AGWN vector
eps = zeros(Ntargets,1);
delta = zeros(2*Ntargets,1);

% create simulated radar doppler measurements
radar_doppler =  simulateRadarDoppler3D( velocity, radar_azimuth, ...
    radar_elevation, eps, delta);

radar_data = [(1:Ntargets)', radar_azimuth, radar_elevation, radar_doppler]

[ model_bruteforce, vhat_all ] = getBruteForceEstimate3D( radar_doppler', ...
    radar_azimuth', radar_elevation');
fprintf('Brute-Force Velocity Profile Estimation\n');
disp([velocity, model_bruteforce])

% get MLESAC (Max. Likelihood RANSAC) model and inlier set
[ model_mlesac, inlier_idx ] = MLESAC_3D( radar_doppler', ...
    radar_azimuth', radar_elevation', sampleSize, maxDistance );
fprintf('MLESAC Velocity Profile Estimation\n');
disp([velocity, model_mlesac])
fprintf('MLESAC Number of Inliers\n');
disp(sum(inlier_idx));

% get 3D Orthogonal Distance Regression (ODR) estimate - MLESAC seed
weights = (1/sigma_vr)*ones(Ntargets,1);
delta = normrnd(0,sigma_theta,[2*Ntargets,1]);
[ model_odr_3D, beta_3D, cov_3D ] = ODR_3D( radar_doppler', radar_azimuth', ...
    radar_elevation', d, model_mlesac, delta, weights );
fprintf('ODR Velocity Profile Estimation - MLESAC seed\n');
disp([velocity, model_odr_3D])

%% Plot Data

sz = 5;
scale = 0;

figure(1)
scatter3(ptcloud(:,1),ptcloud(:,2),ptcloud(:,3),sz,'b','filled');
title('3D Pointcloud - Radar Doppler Measurements','Interpreter','latex')
xlabel('X [m]','Interpreter','latex');
ylabel('Y [m]','Interpreter','latex');
zlabel('Z [m]','Interpreter','latex');
hold on;

for i=1:Ntargets
    % plot radius line
    plot3([0 ptcloud(i,1)], [0 ptcloud(i,2)], [0 ptcloud(i,3)],':','Color','k');
    
    % generate radar doppler vector using unit normal of radius vector
    vec = ptcloud(i,:)' - zeros(3,1);
    unit_vec = vec./norm(vec);
    doppler_vec = radar_doppler(i)*unit_vec;
    
    vrx = doppler_vec(1);
    vry = doppler_vec(2);
    vrz = doppler_vec(3);
%     disp([vrx, vry, vrz]);
    
%     quiver3(ptcloud(i,1),ptcloud(i,2),ptcloud(i,3),-vrx,-vry,-vrz, ...
%         scale,'--');

    % use azimuth and elevation angles to define doppler components...
    % should be identical to above method
    vrx = radar_doppler(i)*cos(radar_azimuth(i))*cos(radar_elevation(i));
    vry = radar_doppler(i)*sin(radar_azimuth(i))*cos(radar_elevation(i));
    vrz = radar_doppler(i)*sin(radar_elevation(i));
%     disp([vrx, vry, vrz]);
%     fprintf('\n')
    
    % plot radar doppler vector
    quiver3(ptcloud(i,1),ptcloud(i,2),ptcloud(i,3),-vrx,-vry,-vrz,scale);
end