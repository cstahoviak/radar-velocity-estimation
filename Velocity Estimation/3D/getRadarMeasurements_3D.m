function [true_doppler, true_azimuth, true_elevation, radar_doppler, ...
    radar_azimuth, radar_elevation] = getRadarMeasurements_3D( Ntargets, ...
    model, radar_angle_bins, sigma_vr, type )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if strcmp(type,'angles')
    % simulated target azimuth angles
    min_azimuth = -70;    % [deg]
    max_azimuth = 70;     % [deg]
    true_azimuth = (max_azimuth - min_azimuth).*rand(Ntargets,1) + min_azimuth;
    true_azimuth = deg2rad(true_azimuth);
    
    % simulated target elevation angles
    min_elevation = -70;    % [deg]
    max_elevation = 70;     % [deg]
    true_elevation = (max_elevation - min_elevation).*rand(Ntargets,1) + min_elevation;
    true_elevation = deg2rad(true_elevation);
elseif strcmp(type,'points')
    ptcloud = generatePointcloud3D( Ntargets );
    
    radar_x = ptcloud(:,1);
    radar_y = ptcloud(:,2);
    radar_z = ptcloud(:,3);
    
    range = sqrt(radar_x.^2 + radar_y.^2 + radar_z.^2);
    r_xy = sqrt(radar_x.^2 + radar_y.^2);
    
    true_azimuth = atan(radar_y./radar_x);      % [rad];
%     true_elevation = atan(radar_z./r_xy);       % [rad];
    true_elevation = asin(radar_z./range);     % [rad];
%     disp([true_elevation, true_elevation2, true_elevation - true_elevation2])
    
else
    true_azimuth = zeros(Ntargets,1);
    true_elevation = zeros(Ntargets,1);
end

radar_azimuth   = zeros(Ntargets,1);
radar_elevation = zeros(Ntargets,1);

% bin angular data
for i=1:Ntargets
    % bin azimuth value - could also add noise
    [~, bin_idx] = min( abs(radar_angle_bins - true_azimuth(i)) );
    radar_azimuth(i) = radar_angle_bins(bin_idx);
    
    % bin elevation value - could also add noise
    [~, bin_idx] = min( abs(radar_angle_bins - true_elevation(i)) );
    radar_elevation(i) = radar_angle_bins(bin_idx);
end

% define AGWN vector for doppler velocity measurements
eps = normrnd(0,sigma_vr,[Ntargets,1]);
% eps = ones(Ntargets,1)*sigma_vr;

% get true radar doppler measurements
true_doppler = simulateRadarDoppler3D(model, true_azimuth, ...
    true_elevation, zeros(Ntargets,1), zeros(2*Ntargets,1));

% get noisy radar doppler measurements
radar_doppler =  simulateRadarDoppler3D(model, radar_azimuth, ...
    radar_elevation, eps, zeros(2*Ntargets,1));

end