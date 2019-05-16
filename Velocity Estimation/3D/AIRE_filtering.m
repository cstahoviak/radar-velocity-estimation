function [ idx_AIR ] = AIRE_filtering( data_AIR, thresholds)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% unpack data (into row vectors)
radar_azimuth   = data_AIR(:,1)';   % [rad]
radar_intensity = data_AIR(:,2)';   % [dB]
radar_range     = data_AIR(:,3)';   % [m]
radar_elevation = data_AIR(:,4)';   % [rad]

azimith_thres   = thresholds(1);
intensity_thres = thresholds(2);
range_thres     = thresholds(3);
elevation_thres = thresholds(4);

% range, angle, intensity filtering:
idx_azimuth   = (abs(rad2deg(radar_azimuth)) < azimith_thres);
idx_intensity = (radar_intensity > intensity_thres);
idx_range     = (radar_range > range_thres);
idx_elevation   = (abs(rad2deg(radar_elevation)) < elevation_thres);

% combine filters
idx_AIR = idx_azimuth & idx_intensity & idx_range & idx_elevation;