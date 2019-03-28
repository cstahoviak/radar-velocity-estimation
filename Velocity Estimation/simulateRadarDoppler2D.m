function [ radar_doppler] = simulateRadarDoppler2D( velocity_body, ...
    radar_angle)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% velocity_body -> body-frame vx and vy

% NOTE: This math is correct (for the zero-pitch and zero-roll 2D case).
% For non-zero pitch and roll, either vx or vy will need to be transformed
% into measurement plane of sensor producing the data (either forward or
% lateral facing radar).

Ntargets = size(radar_angle,1);  % column vector
radar_doppler = zeros(Ntargets,1);

for i=1:Ntargets
    radar_doppler(i) = velocity_body(1)*cos(radar_angle(i)) + ...
        velocity_body(2)*sin(radar_angle(i));
end

end

