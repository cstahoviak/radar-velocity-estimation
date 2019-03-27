function [ model, v_hat_nonZero ] = getBruteForceEstimate_fwd( ...
    radar_doppler, radar_angle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% number of valid targets in scan
Ntargets = size(radar_doppler,2);
iter = (Ntargets-1)*Ntargets/2;

if Ntargets > 1
%     disp('HERE 0')
    % initialize velocity estimate vector
    v_hat = zeros(2,iter);

    k = 1;
    for i=1:Ntargets-1
        for j=(i+1):Ntargets

        % solve uniquely-determined problem for pair of targets (i,j)
        M = [cos(radar_angle(i)), sin(radar_angle(i));
             cos(radar_angle(j)), sin(radar_angle(j))];

        if cond(M) < 100
            v_hat(:,k) = M\[radar_doppler(i); radar_doppler(j)];
    %         fprintf('targets (%d,%d):  \t[vx, vy] = [%.4f, %.4f]\t angle: [%.4f, %.4f]\n', i, j, ...
    %             v_hat(1,k), v_hat(2,k), rad2deg(radar_angle(i)), ...
    %             rad2deg(radar_angle(j)));
        else
            v_hat(:,k) = zeros(2,1);
    %         fprintf('targets (%d,%d):  \tcond(M) = %f\t\t angle: [%.4f, %.4f]\n', ...
    %             i, j, cond(M), rad2deg(radar_angle(i)), rad2deg(radar_angle(j)))
        end

        % TODO: apply rotation to v_y component to transform the v_y estimate
        % into the correct frame. This is because the forward facing sensor is
        % aligned with the body-frame x-axis, and the the v_y estimate computed
        % above lives in the plane defined normal to the sensor which is not
        % the same as the body-frame y-direction.
        % -> need to write quaternion2DCM for pitch/roll ONLY

        k = k+1;
        end 
    end
    
    if sum(sum(v_hat)) == 0
%         disp('HERE 1')
        % there exists no unique solution to the uniquely-determined
        % problem for any two targets in the scan. This is the result of M
        % being close to singular for all pairs of targets, i.e. the
        % targets have identical angular locations.
        v_hat_nonZero = zeros(2,1);
    end
      
else
%     disp('HERE 2')
    % cannot solve uniquely-determined problem for a single target
    % (solution requires 2 non-identical targets)
    v_hat_nonZero = zeros(2,1);
end

if ( Ntargets > 2 ) && ( sum(sum(v_hat)) ~= 0 )
%     disp('HERE 3')
    % if there are more than 2 targets in the scan (resulting in a minimum
    % of 3 estimates of the velocity vector), AND those targets produced
    % non-singular solutions to the uniquely-determined problem
    
    % remove zero values from estimate
    idx_nonZero = (v_hat(1,:) ~= 0);
    v_hat_nonZero = v_hat(:,idx_nonZero);

    % remove 2sigma outliers from data and return sample mean as model
    w = 0;      % weighting scheme
    variance = var(v_hat_nonZero,w,2);  % sample variance
    sigma = sqrt(variance);             % sample std. dev.
    mu = mean(v_hat_nonZero,2);         % sample mean
    
    if sigma(1) > 0
%         disp('HERE 4')
        % 2 targets will result in a single solution, and a variance of 0.
        % 2sigma inliers should only be identified for more than 2 targets.
  
        % inlier_idx = (v_hat(1,:) < 2*sigma(1) & v_hat(1,:) ~= 0 );
        inlier_idx = (abs(v_hat_nonZero(1,:)-mu(1)) < sigma(1));
        model = mean(v_hat_nonZero(:,inlier_idx),2);
        
    else
%         disp('HERE 5')
        % there are more than 2 targets in the scan, but the resulting
        % estimates of v_hat for those Ntargets result in identical
        % estimates and thus a variance of zero.
        
       model = mean(v_hat_nonZero,2);
    end
    
elseif ( Ntargets > 1 ) && ( sum(sum(v_hat)) ~= 0 )
%     disp('HERE 6')
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a non-singular matrix M
    model = v_hat; 
    v_hat_nonZero = v_hat;
    
elseif ( Ntargets > 1 ) && ( sum(sum(v_hat)) == 0 )
%     disp('HERE 7')
    % there are 2 targets in the scan, AND their solution to the
    % uniquely-determined problem produced a singular matrix M, i.e. the
    % targets have identical angular locations.
    model = NaN*ones(2,1);
    v_hat_nonZero = NaN*ones(2,1);
     
else
%     disp('HERE 8')
    % there is a single target in the scan, and the solution to the
    % uniquely-determined problem is not possible
    model = v_hat_nonZero;
end

end

