function [true_angle, true_doppler, radar_angle, radar_doppler ] = ...
    getRadarMeasurements( Ntargets, model, radar_angle_bins, sigma_vr )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

radar_angle = zeros(Ntargets,1);

% simulated true target angles
min_angle = deg2rad(-75);    % [deg]
max_angle = deg2rad(75);     % [deg]
true_angle = (max_angle-min_angle).*rand(Ntargets,1) + min_angle;

% bin angular data
for i=1:Ntargets
    [~,bin_idx] = min( abs(radar_angle_bins - true_angle(i)) );
    radar_angle(i) = radar_angle_bins(bin_idx);
end

% define AGWN vector for doppler velocity measurements
eps = normrnd(0,sigma_vr,[Ntargets,1]);

% get true radar doppler measurements
true_doppler = simulateRadarDoppler2D(model, true_angle, ...
    zeros(Ntargets,1), zeros(Ntargets,1));

% get noisy radar doppler measurements
radar_doppler =  simulateRadarDoppler2D(model, radar_angle, ...
    eps, zeros(Ntargets,1));

end

