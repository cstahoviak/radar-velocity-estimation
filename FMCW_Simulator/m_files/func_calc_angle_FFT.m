function [angle_FFT_power, angle_of_arrival_deg] = func_calc_angle_FFT(I_input, Q_input, Npts)

% This routine calculates the angle FFT using an Npts long FFT

% Input vecotrs are padded with zeros to improve output angle resolution.

% Inputs:  
% I_input      - Real part of array, length less than Npts
% Q_input      - Imaginary part of array, length less than Npts
% Npts         - length of calculated FFT (zero padded)

% Outputs:
% angle_FFT_power       - magnitude of angle_FFT
% angle_of_arrival_deg  - Angle of arrival (degrees) for each bin in angle_FFT

% updated: 07-July-2018
% ========================================================================

%% Define the angles for each bin of the angle_FFT

% define the spacing between each FFT bin in radians
%Npts           = 64;
delta_radian   = (2*pi)/(Npts);
angle_bin_rad = -1*pi + ((1:Npts)-1) * delta_radian;
%angle_bin_deg = angle_bin_rad * (180/pi);

%% Define the angle of arrival for each bin of angle_FFT

% This accounts for delta_phi = k*(lambda/2) sin(theta)
% delta_phi = (2*pi/lambda)*(lambda/2) sin(theta) = pi*sin(theta)
% where theta is the angle of arrival

angle_of_arrival_deg    = ones(1,Npts) .* NaN;
for c = 1:Npts
   if(abs(angle_bin_rad(c) / pi) <= 1)
      angle_of_arrival_deg(c) = real(asin(angle_bin_rad(c)/pi) * (180/pi));
   end % end if(angle < 0)
end % end for c loop

%% Pad the input arrays with zeros

m                 = length(I_input);

I_input_pad       = zeros(Npts,1);
I_input_pad(1:m)  = I_input;
Q_input_pad       = zeros(Npts,1);
Q_input_pad(1:m)  = Q_input;
   
%% Do the complex FFT

% Keep only the power of the computed FFT
[~, IQ_fft_power] = func_calc_complex_FFT(I_input_pad, Q_input_pad);   

angle_FFT_power   = IQ_fft_power;

