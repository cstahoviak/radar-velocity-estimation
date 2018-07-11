function [noise_fft, eof_flag] = func_read_noise_profile(fid, radar_model, tag_length)

% This function reads the noise profile, which is an array of points at 
% maximum Doppler (maximum speed objects). 

% Value: This is the same format as range profile but the profile is at 
% the maximum Doppler bin (maximum speed objects). In general for 
% stationary scene, there would be no objects or clutter at maximum speed 
% so the range profile at such speed represents the receiver noise floor.

% Binary values saved in different Q formats, depending on radar model
% In XWR16xx the points represent the sum of log2 magnitudes of received 
% antennas, expressed in Q8 format. 
% In XWR14xx the points represent the average of log2 magnitudes of 
% received antennas, expressed in Q9 format.

% updated: 03-July-2018
% =======================================================================

%% Determine Q format

if(radar_model > 15)
   Q_format    = 2^8;
else
   Q_format    = 2^9;
end % end if(radar_model > 15)

%% Deterine how many samples to read

% The length is given by:
% tag_length = (range FFT size) x (size of unint16_t)
%   1024 = (512) x (2) 

num_fft_size   = round(tag_length ./ 2);

%% Define the output array

noise_fft      = ones(num_fft_size,1) .* NaN;
%noise_raw_fft  = ones(num_fft_size,1) .* NaN;

%% Reset the eof_flag

eof_flag       = 0;

%% Read each value of noise_fft

for r = 1:num_fft_size
   
   read_value        = fread(fid,1,'uint16');
   if(~isempty(read_value))
      noise_fft(r)   = (read_value) ./ Q_format;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))
   
end % end for r loop

