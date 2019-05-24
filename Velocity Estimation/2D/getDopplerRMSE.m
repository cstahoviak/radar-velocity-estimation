function [ RMSE, error ] = getDopplerRMSE( vhat, radar_time_second, ...
    twist_time_second, twist_linear_body, dim)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% dim - dimension of velocity estimate vector, 2D or 3D

% derivative used to find "spikes" in body-frame velocity
deriv =  abs(diff(vecnorm(twist_linear_body,2,2)));

NScans = size(vhat,1);
error = zeros(NScans,3);

for i=1:NScans
    
    % get index of closest Vicon velocity estimate to current time step
    [~,idx] = min( abs(twist_time_second - radar_time_second(i)) );
    
    if isnan(vhat(i,:))
%         error(i,:) = zeros(1,2);
        error(i,:) = twist_linear_body(idx,1:dim);
    else
        if norm(twist_linear_body(idx,:)) < 2.2
            error(i,:) = vhat(i,:) - twist_linear_body(idx,1:dim);
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

