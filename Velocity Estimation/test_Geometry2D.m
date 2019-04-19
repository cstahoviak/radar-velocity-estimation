%% Header

%%% Filename:   test_Geometry2D.m
%%% Author:     Carl Stahoviak
%%% Created:    03/26/2019  

clc;
close all;

format compact

new_data = true;

%% Main

% number of simulated targets
Ntargets = 50;

if new_data
    
    clearvars -except new_data Ntargets
    
    % hard-coded values
    velocity = [1.25, 0.75]'
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
end

% define AGWN vector
eps = zeros(Ntargets,1);
delta = zeros(Ntargets,1);

% create simulated radar doppler measurements
radar_doppler =  simulateRadarDoppler2D(velocity, deg2rad(radar_angle), ... 
    eps, delta);

radar_data = [(1:Ntargets)', radar_angle, radar_doppler]

[ model, vhat_all ] = getBruteForceEstimate( radar_doppler', ...
    deg2rad(radar_angle)', 100);

fprintf('Brute-Force Velocity Profile Estimation\n');
disp(model)

%% Plot Data

sz = 5;
scale = 0;

figure(1)
scatter(ptcloud(:,1),ptcloud(:,2),ptcloud(:,3),sz,'b','filled');
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
    disp([vrx, vry, vrz]);
    
%     quiver3(ptcloud(i,1),ptcloud(i,2),ptcloud(i,3),-vrx,-vry,-vrz, ...
%         scale,'--');

    % use azimuth and elevation angles to define doppler components...
    % should be identical to above method
    vrx = radar_doppler(i)*cos(radar_azimuth(i))*cos(radar_elevation(i));
    vry = radar_doppler(i)*sin(radar_azimuth(i))*cos(radar_elevation(i));
    vrz = radar_doppler(i)*sin(radar_elevation(i));
    disp([vrx, vry, vrz]);
    fprintf('\n')
    
    % plot radar doppler vector
    quiver3(ptcloud(i,1),ptcloud(i,2),ptcloud(i,3),-vrx,-vry,-vrz,scale);
end

