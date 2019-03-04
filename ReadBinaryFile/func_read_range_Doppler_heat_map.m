function [range_Doppler_heat_map, eof_flag] = func_read_range_Doppler_heat_map(fid, ...
   num_Doppler_bins, tag_length)

% This function reads the range-Doppler heat map.

% updated: 03-July-2018
% =======================================================================

%% Deterine how many samples to read

% Length  = (range FFT size) X (number of Doppler Velocity bins)(size of uin16)

% The value is uint16, which is 2 Bytes.

% The length is given by:
% tag_length = (range FFT size) x (num_Doppler_bins)(2 bytes)
% 131072 = (512) * (128*2)

num_fft_size   = round(tag_length ./ (num_Doppler_bins * 2));

%% Reset the eof_flag

eof_flag       = 0;

%% Define the output array

range_Doppler_heat_map = ones(num_fft_size,num_Doppler_bins) .* NaN;

%% Read each value in range-Doppler matrix

for r = 1:num_fft_size
   for c = 1:num_Doppler_bins

      % first value is imaginary part, second value is real part
      read_value        = fread(fid,1,'int16');
      if(~isempty(read_value))
         range_Doppler_heat_map(r,c)   = read_value;
      else
         eof_flag       = 1;
      end % end if(~isempty(read_value))

   end % end for c loop
   
end % end for r loop

