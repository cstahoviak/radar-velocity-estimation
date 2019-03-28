function [ model, v_hat_all ] = getBruteForceEstimate_fwd( ...
    radar_doppler, radar_angle, conditionNum_thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% number of targets in scan
Ntargets = size(radar_doppler,2);
iter = (Ntargets-1)*Ntargets/2;

if Ntargets > 1
    % initialize velocity estimate vector
    v_hat = zeros(2,iter);

    k = 1;
    for i=1:Ntargets-1
        for j=(i+1):Ntargets
            
            doppler = [radar_doppler(i), radar_doppler(j)];
            angle = [radar_angle(i), radar_angle(j)];
            
            v_hat(:,k) = doppler2BodyFrameVelocities( doppler, ...
                angle, conditionNum_thres);

            % TODO: apply rotation to v_y component to transform the v_y
            % estimate into the correct frame. This is because the forward
            % facing sensor is aligned with the body-frame x-axis, and the
            % v_y estimate computed above lives in the plane defined normal
            % to the sensor which is not the same as the body-frame
            % y-direction.
            % -> need to write quaternion2DCM for pitch/roll ONLY

            k = k+1;
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
        v_hat_all = NaN*ones(2,1);
    end
      
else
    % cannot solve uniquely-determined problem for a single target
    % (solution requires 2 non-identical targets)
    v_hat_nonNaN = [];
    v_hat_all = NaN*ones(2,1);
end

if ( Ntargets > 2 ) && ( ~isempty(v_hat_nonNaN) )
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
    k = 1;
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
    
    % remove k-sigma outliers
    idx_inlier = idx_inlier_x & idx_inlier_y;
    model = mean(v_hat_nonNaN(:,idx_inlier),2);
    v_hat_all = v_hat(:,idx_inlier);
    
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