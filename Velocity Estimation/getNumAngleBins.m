function [ numAngleBins, bins ] = getNumAngleBins( radar_angle )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% NOTE: radar_angle should be a row vector

bin_thres = 0.009;      % [rad]

angle_bins = unique(radar_angle);

bins = [];
current_bin = angle_bins(1);
begin_idx = 1;

for i=1:size(angle_bins,2)
    if abs(current_bin - angle_bins(i)) > bin_thres
        % update location of last angle to be averaged
        end_idx = i-1;
        
        % add single averaged value to table
        angle_bin = mean(angle_bins(begin_idx:end_idx));
        bins = [bins; angle_bin];
        
        % set new beginning index
        begin_idx = i;
        
        % set new current bin
        current_bin = angle_bins(i);
    end
        
    if i == size(angle_bins,2)
        % update location of last angle to be averaged
        end_idx = i;
        
        % add single averaged value to table
        angle_bin = mean(angle_bins(begin_idx:end_idx));
        bins = [bins; angle_bin];
    end
end

numAngleBins = size(bins,1);

end