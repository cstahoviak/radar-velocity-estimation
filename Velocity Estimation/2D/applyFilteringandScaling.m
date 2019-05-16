function [ doppler_scaled, idx_AIR ] = applyFilteringandScaling( ...
    radar_data, thresholds_AIR, idx_pre)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% extract radar data (into row vectors)
radar_range     = radar_data(:,1)';
radar_angle     = radar_data(:,2)';
radar_doppler   = radar_data(:,3)';
radar_intensity = radar_data(:,4)';

% apply {angle, intensity, range} 'AIR' filtering
data_AIR = [radar_angle', radar_intensity', radar_range'];
idx_AIR = AIR_filtering( data_AIR, thresholds_AIR );

% apply AIR filtering to doppler data
doppler_filtered = radar_doppler(idx_pre & idx_AIR);

% get 'scaled' velocity - OLD METHOD (mathematically incorrect)
doppler_scaled = doppler_filtered.*(1./cos(radar_angle(idx_pre & idx_AIR)));

end

