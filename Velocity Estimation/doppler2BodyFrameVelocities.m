function [ v_hat ] = doppler2BodyFrameVelocities( radar_doppler, ...
    radar_angle, conditionNum_thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%%% TODO:
%   1. Experiment with using pinv() rather than \ to estimate the model
%   parameters - this will also require tuning of the MLESAC maxDistance
%   parameter. This way I can actually use targets that fall into the same
%   angular bin to estimate (vx, vy) because I've shown that the binning
%   process is noisy (see test_angleBinning.m) and thus it will never be
%   the case that theta_1 == theta_2.

[ numAngleBins, ~ ] = getNumAngleBins( radar_angle );

if numAngleBins > 1
   % solve uniquely-determined problem for pair of targets (i,j)
   M = [cos(radar_angle(1)), sin(radar_angle(1));
        cos(radar_angle(2)), sin(radar_angle(2))];
     
    v_hat = M\[radar_doppler(1); radar_doppler(2)];
else
    disp([numAngleBins, radar_angle(1), radar_angle(2)])
    v_hat = NaN*ones(2,1);
end

end

