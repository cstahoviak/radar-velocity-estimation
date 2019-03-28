function [ model, v_hat_nonNaN ] = getBruteForceEstimate_fwd( ...
    radar_doppler, radar_angle, conditionNum_thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% number of targets in scan
Ntargets = size(radar_doppler,2);
iter = (Ntargets-1)*Ntargets/2;

if Ntargets > 1
%     disp('HERE 0')
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
    
    if isnan(sum(sum(v_hat)))
%         disp('HERE 1')
        % there exists no unique solution to the uniquely-determined
        % problem for any two targets in the scan. This is the result of M
        % being close to singular for all pairs of targets, i.e. the
        % targets have identical angular locations.
        v_hat_nonNaN = NaN*ones(2,1);
    end
      
else
%     disp('HERE 2')
    % cannot solve uniquely-determined problem for a single target
    % (solution requires 2 non-identical targets)
    v_hat_nonNaN = NaN*ones(2,1);
end

if ( Ntargets > 2 ) && ( sum(sum(v_hat)) ~= 0 )
%     disp('HERE 3')
    % if there are more than 2 targets in the scan (resulting in a minimum
    % of 3 estimates of the velocity vector), AND those targets produced
    % non-singular solutions to the uniquely-determined problem
    
    % remove NaN values from estimate
    idx_nonNaN = ~isnan(v_hat(1,:));
    v_hat_nonNaN = v_hat(:,idx_nonNaN);

    % remove 2sigma outliers from data and return sample mean as model
    w = 0;      % weighting scheme
    variance = var(v_hat_nonNaN,w,2);  % sample variance
    sigma = sqrt(variance);             % sample std. dev.
    mu = mean(v_hat_nonNaN,2);         % sample mean
    
    if sigma(1) > 0
%         disp('HERE 4')
        % 2 targets will result in a single solution, and a variance of 0.
        % 2sigma inliers should only be identified for more than 2 targets.
        inlier_idx = (abs(v_hat_nonNaN(1,:)-mu(1)) < sigma(1));
        model = mean(v_hat_nonNaN(:,inlier_idx),2);
        
    else
%         disp('HERE 5')
        % there are more than 2 targets in the scan, but the resulting
        % estimates of v_hat for those Ntargets result in identical
        % estimates and thus a variance of zero.
       model = mean(v_hat_nonNaN,2);
    end
    
elseif ( Ntargets > 1 ) && ( sum(sum(v_hat)) ~= 0 )
%     disp('HERE 6')
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a non-singular matrix M
    model = v_hat; 
    v_hat_nonNaN = v_hat;
    
elseif ( Ntargets > 1 ) && ( sum(sum(v_hat)) == 0 )
%     disp('HERE 7')
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a singular matrix M, i.e. the
    % targets have identical angular locations.
    model = NaN*ones(2,1);
    v_hat_nonNaN = NaN*ones(2,1);
     
else
%     disp('HERE 8')
    % there is a single target in the scan, and the solution to the
    % uniquely-determined problem is not possible
    model = v_hat_nonNaN;
end

end