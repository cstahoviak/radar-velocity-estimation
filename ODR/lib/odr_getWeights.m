function [ weights ] = odr_getWeights( radar_intensity, sigma_vr, int_range )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% expects radar_intensity as a row vector
Ntargets = size(radar_intensity,2);

max_intensity = int_range(1);   % [dB]
min_intensity = int_range(2);   % [dB]

if min(radar_intensity) < min_intensity || ...
        max(radar_intensity) > max_intensity
    error('Intensity values outside of expected range')
else
    norm_intensity = zeros(1,Ntargets);

    % normalize radar intensity values
    for i=1:Ntargets
        norm_intensity(i) = ( radar_intensity(i) - min_intensity )/ ...
            ( max_intensity - min_intensity );
    end

    % return a column vector of weights
    weights = (norm_intensity./sigma_vr)';

end

