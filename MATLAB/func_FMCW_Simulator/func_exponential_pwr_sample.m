function [I, Q, IQ]  = func_exponential_pwr_sample(input_mean,N_samples)

% This routine returns the I, Q and IQ = I^2 + Q^2 of an exponential pwr
% distribution

% updated: 23-March-2017
% =======================================================================

% This matlab command will pull random samples from a PDF:
%  Y = random('exp',mu,r_max, c_max);
%  will generate a (r_max,c_max) matrix of random samples pulled from an
%  exponential distribution with mean mu.

% The exponential mean is estimated from a gaussan variance:
% exponential_pwr_mean = 2*gaussian_var/Ncoh;
% Thus, given an exponential mean, this is the input gaussan variance:
% gaussian_var = exponential_pwr_mean*Ncoh/2;
% gaussian_std = sqrt(gaussian_var);

% Define the input values:
%exponential_pwr_mean    = 0.5;
%Npts                 = 10000;

% Do the calculations:
gaussian_var = input_mean/2;
gaussian_std = sqrt(gaussian_var);


% Example,
%N_samples   = Ncoh*Npts;
gaussian_mean  = 0;
%input_std   = 0.5;
%input_var   = input_std.^2;

I           = random('norm',gaussian_mean,gaussian_std,1,N_samples);
Q           = random('norm',gaussian_mean,gaussian_std,1,N_samples);

%expected_pwr_mean  = 2*input_var/Ncoh;
%expected_pwr_std   = 2*gaussian_var/Ncoh;
%expected_pwr_var   = expected_pwr_std.^2;

% input power
IQ    = I.^2 + Q.^2;
