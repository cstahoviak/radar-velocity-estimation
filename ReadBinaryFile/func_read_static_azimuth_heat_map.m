function [static_azimuth_heat_map, eof_flag] = func_read_static_azimuth_heat_map(fid, ...
   num_virtual_antennas, tag_length)

% This function reads the azimuth heat map for zero Doppler velocity,
% which is an matrix of complex values for each range and receiver.

% This is the static heat map, which means it is at zero Doppler velocity.

% updated: 03-July-2018
% =======================================================================

%% Deterine how many samples to read

% Length  = (range FFT size) X (number of virtual antennas)(size of complex value)

% The complex value is: imaginary part (int16) real part (int16)
% Each int16 is 2 Bytes. Thus, each complex value is 4 Bytes.

% The length is given by:
% tag_length = (range FFT size) x (num_virtual_antennas)(4 bytes)
%   8192 = (512) * (8*2) 

num_fft_size   = round(tag_length ./ (num_virtual_antennas * 4));

%% Define the output array

static_azimuth_heat_map = ones(num_fft_size,num_virtual_antennas) .* NaN;

%% Reset the eof_flag

eof_flag       = 0;

%% define a constant

sqrt_neg1   = sqrt(-1);

%% read each value of heat map

for r = 1:num_fft_size
   for c = 1:num_virtual_antennas

      % first value is imaginary part, second value is real part
      read_value        = fread(fid,1,'int16');
      if(~isempty(read_value))
         imag_value   = read_value;
      else
         eof_flag       = 1;
      end % end if(~isempty(read_value))
      
      % read the real part
      read_value        = fread(fid,1,'int16');
      if(~isempty(read_value))
         real_value   = read_value;
      else
         eof_flag       = 1;
      end % end if(~isempty(read_value))

      if(eof_flag < 0.5)
         static_azimuth_heat_map(r,c) = real_value + sqrt_neg1 * imag_value;
      end % end if(eof_flag < 0.5)
      
   end % end for c loop
   
end % end for r loop

