function [ RMSE ] = getDopplerRMSE( vhat, radar_time_second, ...
    twist_time_second, twist_linear_body)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

NScans = size(vhat,1);
error = zeros(NScans,2);

for i=1:NScans
    
    if isnan(vhat(i,:))
        error(i,:) = zeros(1,2);
    else
        [~,ix] = min( abs(twist_time_second - radar_time_second(i)) );
        error(i,:) = vhat(i,:) - twist_linear_body(ix,1:2);
    end
    
end

RMSE = sqrt(mean(error.^2,1));
RMSE = RMSE';

end

