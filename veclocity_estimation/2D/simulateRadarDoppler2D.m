function [ radar_doppler] = simulateRadarDoppler2D( velocity_body, ...
    radar_angle, eps, delta)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% velocity_body - body-frame vx and vy
% eps - zero-mean, AGWN to corrupt simulated radar_doppler measurement
% eps - zero-mean, AGWN to corrupt simulated radar_angle measurement

% NOTE: This math is correct (for the zero-pitch and zero-roll 2D case).
% For non-zero pitch and roll, either vx or vy will need to be transformed
% into measurement plane of sensor producing the data (either forward or
% lateral facing radar)... NOT actually true! The "sensing plane" of the
% radar is aligned with the body-frame axes of the quad. Thus no transform
% is required.

Ntargets = size(radar_angle,1);  % column vector
radar_doppler = zeros(Ntargets,1);

for i=1:Ntargets
    
    % add measurement noise distributed as N(0,sigma_theta_i)
    theta = radar_angle(i) + delta(i);

    radar_doppler(i) = velocity_body(1)*cos(theta) + ...
        velocity_body(2)*sin(theta);
    
    % add meaurement noise distributed as N(0,sigma_vr)
    radar_doppler(i) = radar_doppler(i) + eps(i);
end

end

