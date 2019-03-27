function [ radar_doppler] = simulateRadarDoppler2D(velocity, radar_angle)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% NOTE: Pretty sure this math is correct (for the 2D case)

Ntargets = size(radar_angle,1);  % column vector
radar_doppler = zeros(Ntargets,1);

for i=1:Ntargets
    radar_doppler(i) = velocity(1)*cos(radar_angle(i)) + ...
        velocity(2)*sin(radar_angle(i));
end

end

