function [ model, v_hat_all ] = getBruteForceEstimate3D( ...
    radar_doppler, radar_azimuth, radar_elevation)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% number of targets in scan
Ntargets = size(radar_doppler,2);
iter = round(factorial(Ntargets)/(6*factorial(Ntargets-3)));

if Ntargets > 1
    % initialize velocity estimate vector
    v_hat = zeros(3,iter);

    m = 1;
    for i=1:Ntargets-2
        for j=(i+1):Ntargets-1
            for k=(j+1):Ntargets
            
                doppler = [radar_doppler(i), radar_doppler(j), radar_doppler(k)];
                azimuth = [radar_azimuth(i), radar_azimuth(j), radar_azimuth(k)];
                elevation =  [radar_elevation(i), radar_elevation(j), radar_elevation(k)];

                v_hat(:,m) = doppler2BodyFrameVelocities3D( doppler, ...
                    azimuth, elevation);

                m = m+1;
            end
        end 
    end
    
    % identify non-NaN solutions to uniquely-determined problem
    idx_nonNaN = ~isnan(v_hat(1,:));
    v_hat_nonNaN = v_hat(:,idx_nonNaN);
    
    if isempty(v_hat_nonNaN)
        % there exists no unique solution to the uniquely-determined
        % problem for any two targets in the scan. This is the result of M
        % being close to singular for all pairs of targets, i.e. the
        % targets have identical angular locations.
        v_hat_all = NaN*ones(3,1);
    end
      
else
    % cannot solve uniquely-determined problem for a single target
    % (solution requires 2 non-identical targets)
    v_hat_nonNaN = [];
    v_hat_all = NaN*ones(3,1);
end

if ( Ntargets > 3 ) && ( ~isempty(v_hat_nonNaN) )
    % if there are more than 2 targets in the scan (resulting in a minimum
    % of 3 estimates of the velocity vector), AND there exists at least one
    % non-singular solution to the uniquely-determined problem

    % remove 2sigma outliers from data and return sample mean as model
    w = 0;                              % weighting scheme
    variance = var(v_hat_nonNaN,w,2);   % sample variance
    sigma = sqrt(variance);             % sample std. dev.
    mu = mean(v_hat_nonNaN,2);          % sample mean
    
    % 2 targets will result in a single solution, and a variance of 0.
    % k-sigma inliers should only be identified for more than 2 targets.
    k = 2;
    if sigma(1) > 0
        idx_inlier_x = (abs(v_hat_nonNaN(1,:)-mu(1)) < k*sigma(1));
    else
        idx_inlier_x = ones(1,size(v_hat_nonNaN,2));
    end
    
    if sigma(2) > 0
        idx_inlier_y = (abs(v_hat_nonNaN(2,:)-mu(2)) < k*sigma(2));
    else
        idx_inlier_y = ones(1,size(v_hat_nonNaN,2));
    end
    
    if sigma(3) > 0
        idx_inlier_z = (abs(v_hat_nonNaN(3,:)-mu(3)) < k*sigma(3));
    else
        idx_inlier_z = ones(1,size(v_hat_nonNaN,2));
    end
    
    % remove k-sigma outliers
    idx_inlier = idx_inlier_x & idx_inlier_y & idx_inlier_z;
    model = mean(v_hat_nonNaN(:,idx_inlier),2);
    v_hat_all = v_hat(:,idx_inlier);
    
% NOTE: Below this point not adapted for the 3D case
    
elseif ( Ntargets > 1 ) && ( ~isempty(v_hat_nonNaN) )
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a non-singular matrix M
    model = v_hat; 
    v_hat_all = v_hat;
    
elseif ( Ntargets > 1 ) && ( isempty(v_hat_nonNaN) )
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a singular matrix M, i.e. the
    % targets have identical angular locations.
    model = NaN*ones(2,1);
    v_hat_all = NaN*ones(2,1);
     
else
    % there is a single target in the scan, and the solution to the
    % uniquely-determined problem is not possible
    model = NaN*ones(2,1);
end

end