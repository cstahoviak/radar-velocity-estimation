function [chirp_parameters]   = func_defineChirpParameters(parameter_str)

% This function call sets radar operating parameters for different user
% defined input strings

% Inputs:
% parameter_str   = a string that defines which parameter set to use

% Outputs:
% chirp_parameters   - an array of parameters defined as:
%     f_start         = 77 * 10^(9);   % start frequency of sweep (Hz)
%     alpha_chirp     = 70;            % ramp slope, units of MHz/us
%     N_sample        = 256;           % number of samples
%     f_ADC_sample    = 5209000;       % ADC sample rate in Hz
%     N_chirp         = 16;            % number of chirps in Frame
%     Frame_Duration  = 33.333 * 10^-3;% Time of all chirps plus idle time (processing time)
%     chirp_idle_time = 39 * 10^-6;    % time between end-of-chirp and next chirp
%     chirp_ramp_time = 57.14 * 10-6;  % time of whole chirp 

% updated: 15-August-2018
% =======================================================================

% Define the chirp parameters for a given input parameter_str
best_range_res_flag = strcmp(parameter_str,'best_range_res');

if(best_range_res_flag)
   
   f_start         = 77 * 10^(9);   % start frequency of sweep (Hz)
   alpha_chirp     = 70;            % ramp slope, units of MHz/us
   N_sample        = 256;           % number of samples
   f_ADC_sample    = 5209000;       % ADC sample rate in Hz
   N_chirp         = 16;            % number of chirps in Frame
   Frame_Duration  = 33.333 * 10^-3;% Time of all chirps plus idle time (processing time)
   chirp_idle_time = 39 * 10^-6;    % time between end-of-chirp and next chirp
   chirp_ramp_time = 57.14 * 10^-6;  % time of whole chirp
   
   chirp_parameters = [f_start, alpha_chirp, N_sample, f_ADC_sample, ...
      N_chirp, Frame_Duration, chirp_idle_time, chirp_ramp_time];
   
end % end if(best_range_reg_flag)

