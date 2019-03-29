%% Header

%%% Filename:   testGeometry.m
%%% Author:     Carl Stahoviak
%%% Created:    03/26/2019  

clc;
close all;

format compact

new_data = true;

%% Main

% number of simulated targets
Ntargets = 15;

if new_data
    
    clearvars -except new_data Ntargets
    
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
end

% create simulated radar doppler measurements
radar_doppler =  simulateRadarDoppler2D(velocity, deg2rad(radar_angle));

radar_data = [(1:Ntargets)', radar_angle, radar_doppler]

[ model, vhat_all ] = getBruteForceEstimate( radar_doppler', ...
    deg2rad(radar_angle)', 100);
disp(model)

