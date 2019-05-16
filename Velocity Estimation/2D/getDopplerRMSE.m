function [ RMSE ] = getDopplerRMSE( vhat, radar_time_second, ...
    twist_time_second, twist_linear_body)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

NScans = size(vhat,1);
error = zeros(NScans,2);

for i=1:NScans
    
    % get index of closest Vicon velocity estimate to current time step
    [~,ix] = min( abs(twist_time_second - radar_time_second(i)) );
    
    if isnan(vhat(i,:))
%         error(i,:) = zeros(1,2);
        error(i,:) = twist_linear_body(ix,1:2);
    else
        if norm(twist_linear_body(ix,1:2)) < 5
            error(i,:) = vhat(i,:) - twist_linear_body(ix,1:2);
        else
            % find closest value in twist_linear_body vector less than
            % threshold= - really what I need to do is smooth the ground
            % truth velocity data
            
            error(i,:) = zeros(1,2);
        end
    end
end

RMSE = sqrt(mean(error.^2,1));
RMSE = RMSE';

end

