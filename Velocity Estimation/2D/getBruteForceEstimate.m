function [ model, v_hat_all ] = getBruteForceEstimate( ...
    radar_doppler, radar_angle, conditionNum_thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%%% TODO:
% 1. Return k-sigma inlier set of targets

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

            k = k+1;
        end 
    end
%     disp(v_hat')
    % identify non-NaN solutions to uniquely-determined problem
    idx_nonNaN = ~isnan(v_hat(1,:));
    v_hat_nonNaN = v_hat(:,idx_nonNaN);
%     fprintf('size(v_hat_nonNaN,2) = %d\n', size(v_hat_nonNaN,2))
    
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
%     inlier_idx = true(1);
end

% fprintf('mean(v_hat_nonNaN,2) = %f\n', mean(v_hat_nonNaN,2))

if ( Ntargets > 2 ) && ( ~isempty(v_hat_nonNaN) )
    % if there are more than 2 targets in the scan (resulting in a minimum
    % of 3 estimates of the velocity vector), AND there exists at least one
    % non-singular solution to the uniquely-determined problem

    % remove 2sigma outliers from data and return sample mean as model
    w = 1;                              % weighting scheme
    sigma = std(v_hat_nonNaN,w,2);      % sample std. dev.
    mu = mean(v_hat_nonNaN,2);          % sample mean
    
    % 2 targets will result in a single solution, and a variance of 0.
    % k-sigma inliers should only be identified for more than 2 targets.
    k = 2;
    if sigma(1) > 0
        inlier_x = (abs(v_hat_nonNaN(1,:)-mu(1)) < k*sigma(1));
    else
        inlier_x = ones(1,size(v_hat_nonNaN,2));
    end
    
    if sigma(2) > 0
        inlier_y = (abs(v_hat_nonNaN(2,:)-mu(2)) < k*sigma(2));
    else
        inlier_y = ones(1,size(v_hat_nonNaN,2));
    end
    
    % remove k-sigma outliers
    inliers = inlier_x & inlier_y;
    model = mean(v_hat_nonNaN(:,inliers),2);
    v_hat_all = v_hat_nonNaN(:,inliers);
    
%     inlier_idx = getBruteForceInliers( Ntargets, inliers );
    
elseif ( Ntargets > 1 ) && ( ~isempty(v_hat_nonNaN) )
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a non-singular matrix M
    model = v_hat; 
    v_hat_all = v_hat;
%     inlier_idx = true(1,2);
    
elseif ( Ntargets > 1 ) && ( isempty(v_hat_nonNaN) )
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a singular matrix M, i.e. the
    % targets have identical angular locations.
    model = NaN*ones(2,1);
    v_hat_all = NaN*ones(2,1);
    warning('brute-force: 2 targets in the scan at same azimuth bin')
     
else
    % there is a single target in the scan, and the solution to the
    % uniquely-determined problem is not possible
    model = NaN*ones(2,1);
    warning('brute-force: single target in scan')
end

end

function [ inlier_idx ] = getBruteForceInliers( Ntargets, inliers )

iter = size(inliers,2);
inlier_idx = false(1,Ntargets);

k = 1;
for i=1:iter-1
    for j=i+1:iter
        if inliers(k) == true
            inlier_idx(i) = true;
            inlier_idx(j) = true;
        end
        k = k+1;
    end
end

end