function [ vx_hat_mean, vx_hat_max, vx_hat_mean_err, vx_hat_max_err ] = ...
    getForwardVelocityEstimate( doppler_scaled, radar_time, ...
    twist_time_second, twist_linear_body )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% estimate forward velocity - mean & max
if isempty( doppler_scaled )
    vx_hat_mean = 0;
    vx_hat_max  = 0;
else
    % multiply by -1 to get platform velocity, not target velocity
    vx_hat_mean = (-1)*mean(doppler_scaled);
    vx_hat_max  = (-1)*sign(mean(doppler_scaled)) * max(abs(doppler_scaled));
end

% calculate error - handles unequal sample rates between radar and Vicon
% [ d, ix ] = min( abs( x-val ) );    % ix is the "index in 1D array that has closest value to" val
[~,ix] = min( abs( twist_time_second - radar_time ) );
vx_hat_mean_err = abs( vx_hat_mean - twist_linear_body(ix,1) );
vx_hat_max_err  = abs( vx_hat_max - twist_linear_body(ix,1) );

end

