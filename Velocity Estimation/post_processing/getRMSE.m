function [ rmse, error, idx ] = getRMSE( estimate, estimate_time, ...
    truth, truth_time, dim, thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% dim - dimension of velocity estimate vector, 2D or 3D

% derivative used to find "spikes" in body-frame velocity
% deriv =  abs(diff(vecnorm(velocity_gt,2,2)));

% TODO:
% 1. Interpolate between Vicon measurements rather than using closest
% measurement to compute RMSE

NScans = size(estimate,1);
error = zeros(NScans,dim);

for i=1:NScans
    
    % get index of closest Vicon velocity estimate to current time step
    [~,idx] = min( abs(truth_time - estimate_time(i)) );
    
    if abs(estimate_time(i) - truth_time(idx)) > 0.01
        warning('n\getRMSE: groundtruth time and estimate time separated by more than 10ms');
    end
    
    if isnan(estimate(i,:))
%         error(i,:) = zeros(1,2);
        error(i,:) = abs(truth(idx,1:dim));
    else
        % compute absolute value of error
        error(i,:) = abs(truth(idx,1:dim) - estimate(i,:));
        
        if vecnorm(error(i,:)) > thres
            error(i,:) = NaN*ones(1,dim);
        else
            % do nothing - error less than threshold
        end
    end
end

% remove NaN values from error matrix
idx = ~isnan(error(:,1));
error = error(idx,:);

% compute component wise RMSE
rmse = sqrt(mean(error.^2,1));
rmse = rmse';

end

