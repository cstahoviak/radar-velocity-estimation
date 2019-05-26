function [ RMSE, error ] = getDopplerRMSE( velocity_estimate, radar_time, ...
    velocity_time, velocity_gt, dim)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% dim - dimension of velocity estimate vector, 2D or 3D

% derivative used to find "spikes" in body-frame velocity
% deriv =  abs(diff(vecnorm(velocity_gt,2,2)));

NScans = size(velocity_estimate,1);
error = zeros(NScans,3);

% thres = 2.2;
thres = inf;

for i=1:NScans
    
    % get index of closest Vicon velocity estimate to current time step
    [~,idx] = min( abs(velocity_time - radar_time(i)) );
    
    if isnan(velocity_estimate(i,:))
%         error(i,:) = zeros(1,2);
        error(i,:) = velocity_gt(idx,1:dim);
    else
        if norm(velocity_gt(idx,:)) < thres
            error(i,:) = velocity_gt(idx,1:dim) - velocity_estimate(i,:); 
        else
            % find closest value in twist_linear_body vector less than
            % threshold= - really what I need to do is smooth the ground
            % truth velocity data
            
            error(i,:) = zeros(1,dim);
        end
    end
end

RMSE = sqrt(mean(error.^2,1));
RMSE = RMSE';

end

