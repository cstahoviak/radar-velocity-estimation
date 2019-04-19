function [ v_hat ] = doppler2BodyFrameVelocities3D( radar_doppler, ...
    radar_azimuth, radar_elevation, conditionNum_thres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%%% TODO:
%   1. Experiment with using pinv() rather than \ to estimate the model
%   parameters - this will also require tuning of the MLESAC maxDistance
%   parameter. This way I can actually use targets that fall into the same
%   angular bin to estimate (vx, vy) because I've shown that the binning
%   process is noisy (see test_angleBinning.m) and thus it will never be
%   the case that theta_1 == theta_2.

theta = radar_azimuth;
phi = radar_elevation;

[ numAngleBins, ~ ] = getNumAngleBins( radar_azimuth );

if numAngleBins > 1
   % solve uniquely-determined problem for pair of targets (i,j)
   M = [cos(theta(1))/cos(phi(1)), sin(theta(1))/cos(phi(1)), sin(phi(1));
        cos(theta(2))/cos(phi(2)), sin(theta(2))/cos(phi(2)), sin(phi(2));
        cos(theta(3))/cos(phi(3)), sin(theta(3))/cos(phi(3)), sin(phi(3))
        ];
     
    v_hat = M\[radar_doppler(1); radar_doppler(2); radar_doppler(3)];
else
    v_hat = NaN*ones(3,1);
end

end