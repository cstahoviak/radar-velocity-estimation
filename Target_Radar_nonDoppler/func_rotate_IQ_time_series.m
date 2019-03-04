function [I_output, Q_output] = func_rotate_IQ_time_series(I_input, Q_input)

% This routine rotates the I & Q time-series by successive -pi radians.

% Inputs:
% I_input      - input I voltage
% Q_input      - input Q voltage

% Outputs
% I_output     - output I voltage 
% I_output     - output Q voltage

% updated: 09-September-2018
% ========================================================================

% % Set the rotation value:

% Since the Rx signal is always delayed version of Tx, I & Q input will
% have a positive phase progression, or counter clockwise rotation.

% The offset betwee successive samples will be -degrees, to slow down, and
% even make clockwise rotating vectors.

% The offset will be -pi radians for every time step.
phi_offset = (-1)*180;

%% Calculate the amplitude and phase of each samples

[m,n]       = size(I_input);
A           = zeros(m,n);
theta       = zeros(m,n);
theta_deg   = zeros(m,n);

for k = 1:m
   A(k)         = sqrt(I_input(k).^2 + Q_input(k).^2);
   theta(k)     = atan2(Q_input(k),I_input(k));
   theta_deg(k) = theta(k) .*(180/pi);

   %if(theta_deg(k) < 0)
   %   theta_deg(k) = theta_deg(k) + 360;
   %end % end if(theta_deg(k) < 0)
    
end % end for k loop

%% Rotate the samples

new_theta_deg  = zeros(m,n);
I_output       = zeros(m,n);
Q_output       = zeros(m,n);

for k = 1:m
   
   phi_rotate = (k-1)*phi_offset;
   
   new_theta_deg(k)  = theta_deg(k) + phi_rotate;
   
   % construct the I and Q voltages
   I_output(k)    = A(k) * cos(new_theta_deg(k) *pi/180);
   Q_output(k)    = A(k) * sin(new_theta_deg(k) *pi/180);
   
end % end for k loop

%% Test rotation

% % Calculate the ffts
% [IQ_fft_complex, ~] = func_calc_complex_FFT(I_input, Q_input);
%
% [new_IQ_fft_complex, ~] = func_calc_complex_FFT(I_output, Q_output);
%
% %% plot some figures
%
% figure
% plot(abs(IQ_fft_complex),'ko-')
% hold on
% plot(abs(new_IQ_fft_complex),'r*-')
% plot([129 129],[0 0.02],'g','linewidth',2)
% grid on
% axis([0 190 0 0.03])

