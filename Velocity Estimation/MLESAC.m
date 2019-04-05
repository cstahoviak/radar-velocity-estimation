function [ model, inlier_idx ] = MLESAC( radar_doppler, radar_angle, ...
    sampleSize, maxDistance, conditionNum_thres)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% NOTE: radar_angle and radar_doppler are row vectors

% An mxn matrix. Each row represents a single data point
% in the set to be modeled
data = [radar_angle', radar_doppler'];
Ntargets = size(data,1);

[ numAngleBins, ~ ] = getNumAngleBins( radar_angle );

if numAngleBins == 1
    disp('HERE');
end

if (Ntargets >= 5) && (numAngleBins > 1)
    % ransac requires a minimum of 5 targets to operate on
    [ model,inlier_idx ] = ransac(data, @MLESAC_fitFcn, @MLESAC_distFcn, ...
        sampleSize, maxDistance, 'ValidateModelFcn', @validateMSS);

else 
    % ransac requires a minimum of 5 targets to operate on. In the case
    % where there are less than 5 targets, we will use the brute force
    % estimation scheme in place of MLESAC
    [ model, ~ ] = getBruteForceEstimate( radar_doppler, ...
        radar_angle, conditionNum_thres);
    inlier_idx = ones(Ntargets,1);
end

end

function [ model ] = MLESAC_fitFcn( data )
    %%% Compute [vx, vy] from [theta, vr]
    % solve uniquely-determined problem for pair of targets
    
    % need to figure out how to pass this value in...
    conditionNum_thres = 100;

    radar_angle   = data(:,1);    % [rad]
    radar_doppler = data(:,2);    % [m/s]
    
    v_hat = doppler2BodyFrameVelocities( radar_doppler', ...
    radar_angle', conditionNum_thres);
    
    model = v_hat;  % v_hat = [vx_hat; vy_hat]
end

function [ distances ] = MLESAC_distFcn( model, data )
    
    % number of targets in scan
    Ntargets = size(data,1);
    distances = zeros(Ntargets,1);

    radar_angle   = data(:,1);    % [rad]
    radar_doppler = data(:,2);    % [m/s]
    
    % do NOT corrupt measurements with noise
    eps = zeros(Ntargets,1);
    delta = zeros(Ntargets,1);
    
    % radar doppler generative model
    doppler_predicted = simulateRadarDoppler2D(model, ...
        radar_angle, eps, delta);
    
    % is this an appropriate distance function??
%     fprintf('\nDistances:\n')
    for i=1:Ntargets
        distances(i) = sqrt((doppler_predicted(i) - radar_doppler(i))^2);
    end

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

    if isnan(model(1))
        isValid = false;
    else
        isValid = true;
    end
    
%     if isempty(model)
%         disp('HERE')
%         isValid = false;
%     else
%         if isnan(model(1))
%             isValid = false;
%         else
%             isValid = true;
%         end
%     end

end
