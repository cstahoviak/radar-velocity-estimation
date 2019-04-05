function [ v_hat ] = doppler2BodyFrameVelocities( radar_doppler, ...
    radar_angle, conditionNum_thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[ numAngleBins, ~ ] = getNumAngleBins( radar_angle );

if numAngleBins > 1
   % solve uniquely-determined problem for pair of targets (i,j)
   M = [cos(radar_angle(1)), sin(radar_angle(1));
        cos(radar_angle(2)), sin(radar_angle(2))];
     
    v_hat = M\[radar_doppler(1); radar_doppler(2)];
else
    v_hat = NaN*ones(2,1);
end
    
return;

% solve uniquely-determined problem for pair of targets (i,j)
M = [cos(radar_angle(1)), sin(radar_angle(1));
     cos(radar_angle(2)), sin(radar_angle(2))];

if cond(M) < conditionNum_thres
    v_hat = M\[radar_doppler(1); radar_doppler(2)];
else
    v_hat = NaN*ones(2,1);
end

end

