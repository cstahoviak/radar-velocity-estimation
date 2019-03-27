function [ model, inlier_idx ] = MLESAC( radar_doppler, radar_angle )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% NOTE: radar_angle and radar_doppler are row vectors

% define RANSAC parameters
sampleSize = 2;     % problem uniquely-determined for 2 targets
maxDistance = 0.1;   % not yet sure what this should be...

% An mxn matrix. Each row represents a single data point
% in the set to be modeled
data = [radar_angle', radar_doppler'];
Ntargets = size(data,1);

if Ntargets >= 5
    % ransac requires a minimum of 5 targets to operate on
    [ model,inlier_idx ] = ransac(data, @MLESAC_fitFcn, @MLESAC_distFcn, ...
        sampleSize, maxDistance);
else
    % ransac requires a minimum of 5 targets to operate on. In the case
    % where there are less than 5 targets, we will use the brute force
    % estimation scheme in place of MLESAC
    [ model, ~ ] = getBruteForceEstimate_fwd( radar_doppler, radar_angle);
    inlier_idx = ones(Ntargets,1);
end

end

function [ model ] = MLESAC_fitFcn( data )
    %%% Compute [vx, vy] from [theta, vr]
    % solve uniquely-determined problem for pair of targets
    
%     disp(data)

    radar_angle = data(:,1);      % [rad]
    radar_doppler = data(:,2);    % [m/s]

    M = [cos(radar_angle(1)), sin(radar_angle(1));
         cos(radar_angle(2)), sin(radar_angle(2))];
    
    v_hat = M\[radar_doppler(1); radar_doppler(2)];
    
    model = v_hat;  % v_hat = [vx_hat; vy_hat]
end

function [ distances ] = MLESAC_distFcn( model, data )
    
    % number of targets in scan
    Ntargets = size(data,1);
    distances = zeros(Ntargets,1);

    radar_angle = data(:,1);      % [rad]
    radar_doppler = data(:,2);    % [m/s]
    
    doppler_predicted = simulateRadarDoppler2D(model, radar_angle);
    
    % is this an appropriate distance function??
%     fprintf('\nDistances:\n')
    for i=1:Ntargets
        distances(i) = sqrt((doppler_predicted(i) - radar_doppler(i))^2);
    end

end
