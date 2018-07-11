function [range_fft, eof_flag] = func_read_range_profile(fid, radar_model, tag_length)

% This function reads the range profile, which is an array of points at 
% 0th Doppler (stationary objects). 

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

%% Define the output variable and set to NaN

range_fft      = ones(num_fft_size,1) .* NaN;
%disp(['  num_fft_size: ',num2str(num_fft_size)]);

%% Reset the eof_flag

eof_flag       = 0;

%% Read each value of range_fft

for r = 1:num_fft_size
      
   read_value        = fread(fid,1,'uint16');
   if(~isempty(read_value))
      range_fft(r)   = (read_value) ./ Q_format;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))
   %disp(['range index: ',num2str(r),', range fft: ',num2str(range_fft(r))])
   
end % end for r loop

