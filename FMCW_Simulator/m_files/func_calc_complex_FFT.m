function [IQ_fft_complex, fft_power] = func_calc_complex_FFT(I_input, Q_input)

% This routine calculates the complex FFT

% updated: 24-May-2018
% ========================================================================

%% Get the number of points in FFT

Npts  = length(I_input);

%% calculate the complex fft

IQ_fft      = fft(I_input + sqrt(-1)*Q_input);

%% Shift the spectra to put DC in the middle.

IQ_fft_complex = fftshift(IQ_fft);

%% Compute the power in each frequency bin

fft_power     = (real(IQ_fft_complex).^2 + ...
   imag(IQ_fft_complex).^2) ./ (Npts);

%% Test the calculation using Parseval's theorem

% Parseval's theorem says that the power_in equals the power_out.
%Power_in          = (sum(I_input.^2) + sum(Q_input.^2));
%Power_out         = (sum(fft_power));
