function [ eps ] = doppler_residual( model, radar_azimuth, ...
    radar_elevation,  radar_doppler)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Ntargets = size(radar_doppler,1);

doppler_predicted = simulateRadarDoppler3D( model, radar_azimuth, ...
        radar_elevation, zeros(Ntargets,1), zeros(2*Ntargets,1) );

eps = doppler_predicted - radar_doppler;

end

