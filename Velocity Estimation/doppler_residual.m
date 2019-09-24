function [ eps ] = doppler_residual( model, data )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% unpack data (into column vectors)
radar_doppler   = data(:,1);
radar_azimuth   = data(:,2);
radar_elevation = data(:,3);

Ntargets = size(radar_doppler,1);
p = size(model,1);

if p == 2
    doppler_predicted = simulateRadarDoppler2D( model, radar_azimuth, ...
        zeros(Ntargets,1), zeros(Ntargets,1) );
elseif p == 3
    doppler_predicted = simulateRadarDoppler3D( model, radar_azimuth, ...
        radar_elevation, zeros(Ntargets,1), zeros(2*Ntargets,1) );
else
    error('model must be a 2D or 3D vector')
end

% calculate Doppler velocity residual vector
eps = doppler_predicted - radar_doppler;

end

