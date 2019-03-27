function [ idx ] = prefilter( radar_doppler, filter_nonZero )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

% extract all non-NaN values from row
idx_nonNaN = ~isnan(radar_doppler);

% get indices of all non-zero doppler targets
idx_nonZero = (radar_doppler ~= 0);

% combine filters
if filter_nonZero
    idx = idx_nonNaN & idx_nonZero;
else
    idx = idx_nonNaN;
end

end

