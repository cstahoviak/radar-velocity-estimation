function [ model, inlier_idx ] = MLESAC_3D( radar_doppler, radar_azimuth, ...
    radar_elevation, sampleSize, maxDistance)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% NOTE: radar_angle and radar_doppler are row vectors

% An mxn matrix. Each row represents a single data point
% in the set to be modeled
data = [radar_doppler' radar_azimuth', radar_elevation'];
Ntargets = size(data,1);

[ numAngleBins, ~ ] = getNumAngleBins( radar_azimuth );

if (Ntargets >= 5) %&& (numAngleBins > 1)
    % ransac requires a minimum of 5 targets to operate on
    [ model, inlier_idx ] = ransac(data, @MLESAC_fitFcn, @MLESAC_distFcn, ...
        sampleSize, maxDistance, 'ValidateModelFcn', @validateMSS);

else 
    % ransac requires a minimum of 5 targets to operate on. In the case
    % where there are less than 5 targets, we will use the brute force
    % estimation scheme in place of MLESAC
    warning('brute-force estimate used in place of MLESAC')
    [ model, ~ ] = getBruteForceEstimate3D( radar_doppler, ...
        radar_elevation, radar_elevation);
    inlier_idx = ones(Ntargets,1);
end

end

function [ model ] = MLESAC_fitFcn( data )
    %%% Compute [vx, vy] from [theta, vr]
    % solve uniquely-determined problem for pair of targets
    
    % need to figure out how to pass this value in...
    conditionNum_thres = 100;

    radar_doppler   = data(:,1);    % [m/s]
    radar_azimuth   = data(:,2);    % [rad]
    radar_elevation = data(:,3);    % [rad]
    
    model = doppler2BodyFrameVelocities3D( radar_doppler', ...
    radar_azimuth', radar_elevation' );
end

function [ distances ] = MLESAC_distFcn( model, data )
    
    % number of targets in scan
    Ntargets = size(data,1);
    distances = zeros(Ntargets,1);

    radar_doppler   = data(:,1);    % [m/s]
    radar_azimuth   = data(:,2);    % [rad]
    radar_elevation = data(:,3);    % [rad]
    
    % do NOT corrupt measurements with noise
    eps         = zeros(Ntargets,1);
    delta_theta = zeros(Ntargets,1);
    delta_phi   = zeros(Ntargets,1);
    
    % radar doppler generative model
    doppler_predicted = simulateRadarDoppler3D( model, radar_azimuth, ...
        radar_elevation, eps, [delta_theta; delta_phi] );
    
    % is this an appropriate distance function??
%     fprintf('\nDistances:\n')
    distances = sqrt( (doppler_predicted - radar_doppler).^2 );
end

function [ isValid ]  = validateMSS( model, varargin )
    % This function returns true if the model is accepted based on criteria
    % defined in the function. Use this function to reject specific fits.
    
%     persistent count;
%     
%     if isempty(count)
%         count = 1;
%     else
%         count = count + 1;
%     end
%     
%     [m,n] = size(model);
%     disp([count, m, n])
%     disp(model); fprintf('\n')

    if isempty(model)
        error('empty model parameters')
    else
        if isnan(model(1))
            isValid = false;
        else
            isValid = true;
        end
    end

end