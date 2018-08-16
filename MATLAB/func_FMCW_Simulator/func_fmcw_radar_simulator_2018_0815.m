function [radar_target_list]  = func_fmcw_radar_simulator_2018_0815(target_attribute, ...
   antenna_gain_pattern, chirp_parameters)

% This routine simulates a fmcw radar and produces a target list.

% This simulator mimics the ADC voltages collected by the
% AWR1642 evaluation board and the performs the fmcw processing.

% This simulator collects a single burst (or frame) of observations.

% Inputs:
% target_attribute   -  2-D matrix of target attributes. 
%                       Each row is a different target.
%                       Each column is a different target attribute.
%                       Matrix size is (N_targets,5) 
%                          where:
%                       N_targets    - number of targets
%                       column    Description
%                         1       target size (RCS) is in m^2.
%                         2       x position [m/s]
%                         3       y position [m/s]
%                         4       target speed [m/s]
%                         5       target attack angle [degrees]
% antenna_gain_pattern - Lookup table of antenna gain.
%                       The two columns are:
%                       col     Description
%                        1      angle in degrees, 180 = left, 90 = straight in front, 0 = right
%                        2      antenna gain, in linear units.

% Outputs:
% radar_target_list  -  List of 64 targets   
%                       One target per row.
%                       columns are defined as:
%                       column    Description
%                         1       SNR - signal to noise ratio
%                         2       x distance (postive to the right of radar)
%                         3       y distance 
%                         4       z distance (positive is above radar)
%                         5       Doppler velocity (positive = target approaching radar)

% Steps:
% Section I.   Define the radar operating parameters
% Section II.  Transmit multiple chirps
% Section III. Process the ADC voltages

%% Section I.  Define the radar operating parameters
% ========================================================================

% The chirp parameters are defined in this function call:
%[chirp_parameters]   = func_defineChirpParameters( 'best_range_res');

% Decode the chirp parameter array into the proper single variable names:

% The parameters needed from user to define radar operating parameters are:
f_start         = chirp_parameters(1); % start frequency of sweep (Hz)
alpha_chirp     = chirp_parameters(2); % ramp slope, units of MHz/us
N_sample        = chirp_parameters(3); % number of samples
f_ADC_sample    = chirp_parameters(4); % ADC sample rate in Hz
N_chirp         = chirp_parameters(5); % number of chirps in Frame
%Frame_Duration  = chirp_parameters(6); % Time of all chirps plus idle time (processing time)
chirp_idle_time = chirp_parameters(7); % time between end-of-chirp and next chirp
chirp_ramp_time = chirp_parameters(8); % time of whole chirp 

% The time between chirps (Chirp Cycle Time)
Tipp                       = chirp_idle_time + chirp_ramp_time;

%% Define other radar operating parameters

% Define Transmitted power
Pt_radar                   = 0.00001;             % Watts (0.01 W = 10 mW)
% express in dBm: dBm = 10*log10(Pt_radar/0.001)
% 0.01  W = 10 mW = 10 dBm
% 0.001 W =  1 mW =  0 dBm

% Define the operating frequency
% Use 75 GHz to define lambda
% define the start frequency

Freq        = 75 * 10^9;       % operating Frequency (Hz)
c_light     = 3*10^8;
lambda      = (c_light)/Freq;    % operating wavelength (m)

%f_start     = 77 * 10^(9);    % start frequency of sweep (Hz)

%% Define the chirp parameters

% set the chirp slope rate. (hardware dependent)
%alpha_chirp    = 70;    % units of MHz/us

% The bandwidth and chirp duration are determine using:
% alpha_chirp  = B_chirp/tau_chirp
% B_chirp      = tau_chirp * alpha

% tau_chirp is the duration samples are collected
%N_sample    = 256;
%f_ADC_sample = 5209000; % ADC sample rate in Hz
t_ADC_sample = 1./f_ADC_sample;
%t_sample    = 200 * 10^(-9);  % Sample rate = 1/t_s = 1/200 ns = 5 MHz
tau_chirp   = N_sample * t_ADC_sample; % units of seconds

% Number of range gates is N_sample/2
N_range     = N_sample/2;

% define the index that will have zero range in range_FFT
DC_index    = N_sample/2 + 1;

% Estimate the chirp bandwidth
% need to convert tau_chirp from sec to us
B_chirp_MHz = (tau_chirp./(10^-6)) * alpha_chirp; % units of MHz

% convert to units of Hz
B_chirp_Hz  = B_chirp_MHz .* 10^6; % units of Hz

% Range resolution
% delta_R = c / (2*B)
delta_R_chirp = c_light / (2*B_chirp_Hz);

% Define the true range to each range gate
distance_each_range_gate  = (0:N_range-1) .* delta_R_chirp;

%% Define the number of Transmitters and Recievers

N_Tx  = 2;
N_Rx  = 4;

%% Define the antenna properties


%   Tx2    0    Tx1        Rx4   Rx3   Rx2   Rx1
%    |  L  |   L | baseline | L/2 | L/2 | L/2 |

% 0         = origin
% baseline  = 8.7 mm
baseline  = 0.0087; % expressed in meters
% L         = lambda

% Virtual Rx
%    Rx8   Rx7   Rx6   Rx5   Rx4   Rx3   Rx2   Rx1
%     | L/2 | L/2 | L/2 | L/2 | L/2 | L/2 | L/2 |
%     |       Tx2       |     |  Tx1            |

% The range to the target is based off a wave originating at (0,0).
% Define the offset from x = 0, for each Tx and Rx.

offset_Tx2     = -lambda;
offset_Tx1     =  lambda;
offset_Tx(1)   = offset_Tx1;
offset_Tx(2)   = offset_Tx2;

offset_Rx4     =  lambda + baseline;
offset_Rx3     =  lambda + baseline + lambda/2;
offset_Rx2     =  lambda + baseline + 2*(lambda/2);
offset_Rx1     =  lambda + baseline + 3*(lambda/2);
offset_Rx(1)   = offset_Rx1;
offset_Rx(2)   = offset_Rx2;
offset_Rx(3)   = offset_Rx3;
offset_Rx(4)   = offset_Rx4;

% The range to a target at (x,y) will be the sum of the ranges from Tx and
% back to Rx.

% R_Tx1 = sqrt( (x - offset_Tx1)^2 + y^2 );
% R_Tx2 = sqrt( (x - offset_Tx2)^2 + y^2 );
% R_Rxi = sqrt( (x - offset_Rxi)^2 + y^2 );

%% Define the time between chirps

% The time needs to be greater than tau_chirp with some overhead.
% tau_chirp is about 52 x 10^-6...round up to 100 microseconds (us)

% Time between chirps is called: Chirp Cycle Time:
%Tipp                       = chirp_idle_time + chirp_ramp_time;

%% Define the number of chirps per frame

% The number of chirps will be used in the Doppler_FFT
% an input from chirp_parameters
%N_chirp                    = 64;

%% Define Doppler parameters

%f_nyquist                  = 1/ (2*Tipp);
delta_v                    = (lambda/(4*Tipp))*(2/N_chirp);
Vd                         = delta_v*((-1)*(N_chirp/2):1:(N_chirp/2)-1);

%% Calculate the Noise power

% The noise power is given as:
% Pn  = k * Te_sys * B
% Pn  = k * (F_linear - 1) * T0 * B

% For these radars, the all have the same noise figure and bandwidth
F_dB                       = 30;            % dB
F_linear                   = 10.^(F_dB/10);  % linear

%B_Hz                       = 1./tau;        % Hz (for pulse system
B_Hz                       = 1./tau_chirp;        % Hz

k                          = 1.38 .* 10^-23;    % Watt-sec/K
T0                         = 290;               % degree K

% Calculate the noise power
Pn_linear                  = k * (F_linear - 1) * T0 * B_Hz;

% Express the noise power in dBm
%Pn_dBm                     = 10.*log10(Pn_linear./10^-3);

% Should system noise be added to data-cube
add_system_noise_flag      = 1;  % 1 = add noise, 0 = no noise

% SNR Threshold  
%SNR_threshold              = 1; % [dB] threshold for plotting

%% Display Radar Attributes

% disp(' ')
% disp('Defined Radar Attributes')
% disp('=======================')
% disp(['Pt:                    ',num2str(10*log10(Pt_radar/0.001)),' dBm']);
% % get the gain at bore sight
% [~,index]                     = min(abs(G_theta_range));
% G_radar_bore_sight_dB         = 10.*log10(G_antenna_pattern_lin(index));
% disp(['Gain(90 deg):          ',num2str(G_radar_bore_sight_dB),' dB']);
% disp(['Reference Freq:        ',num2str(Freq./10^9),' GHz']);
% disp(['wavelength:            ',num2str(lambda),' m']);
% disp(['wavelength:            ',num2str(lambda./10^-3),' mm']);
% disp(['Start Freq:            ',num2str(f_start./10^9),' GHz']);
% disp(['Chirp Slope, alpha:    ',num2str(alpha_chirp),' MHz/us']);
% disp(['Sample Time, t_s:      ',num2str(t_sample./10^(-9)),' ns']);
% disp(['Sample Rate, f_s:      ',num2str(1/(t_sample)./10^(6)),' MHz']);
% disp(['N_samples per chirp:   ',num2str(N_sample),' ']);
% disp(['Chirp duration:        ',num2str(tau_chirp./10^(-6)),' us']);
% disp(['Chirp bandwidth:       ',num2str(B_chirp_MHz./1000),' GHz']);
% disp(['Range resolution:      ',num2str(delta_R_chirp./(10.^-2)),' cm']);
% disp(['Time between chirps:   ',num2str(Tipp ./ 10^-6),' microseconds']);
% disp(['# chirps per frame:    ',num2str(N_chirp),' ']);
% disp(' ')
% disp(['Pn (noise power):      ',num2str(Pn_dBm),' dBm']);
% disp(['add_system_noise_flag: ',num2str(add_system_noise_flag),', 1 = add noise, 0 = no noise added']);

%% Section II.   Transmit multiple chirps
% ===========================================

%% Define the arrays and matrices

% Define time of the start of each chirp
time_transmit  = ones(N_chirp,1) .* NaN;

% Define the ADC times collected during the chirp
ADC_t_s  = (0:1:N_sample-1) .* t_ADC_sample;

% Define I and Q voltages for each (Tx,Rx, Chirp, ADC sample)
ADC_I_voltage_TxRx    = zeros(N_Tx,N_Rx,N_chirp,N_sample);
ADC_Q_voltage_TxRx    = zeros(N_Tx,N_Rx,N_chirp,N_sample);

% Determine how many targets are in scene
[N_target,~]  = size(target_attribute);

%% Process each chirp

for k    = 1:N_chirp
   
   %% Save the time of this chirp
   
   time_transmit(k)    = Tipp * k;
   
   if(rem(k,1000) == 0)
      disp(['processing pulse #',num2str(k),' out of ',num2str(N_chirp),' pulses...'])
   end % end if(rem(i,1000) == 0)
         
   %% Process multiple targets
   
   for e = 1:N_target
      
      %% Define the new target location
      
      % Don't need to keep the old location of the target
      % Only need to know the current location of the target
      
      % get the target size
      target_RCS  = target_attribute(e,1);
      
      % calculate the target gradient in x and y directions
      target_mx    = cos((pi/180) .* target_attribute(e,5));
      target_my    = sin((pi/180) .* target_attribute(e,5));
      
      % update the target's location:
      % x_new  = x_old + mx*(speed*dT);
      % y_new  = y_old + my*(speed*dT);
      
      target_x  = target_attribute(e,2) + target_mx * (target_attribute(e,4) * Tipp);
      target_y  = target_attribute(e,3) + target_my * (target_attribute(e,4) * Tipp);
      
      % save these new locations for the next chirp
      target_attribute(e,2)  = target_x;
      target_attribute(e,3)  = target_y;
            
      %% Process each Transmiter
      
      for i = 1:N_Tx
         
         % calculate the range from Tx to the target
         % R_Tx1 = sqrt( (x - offset_Tx1)^2 + y^2 );
         element_x      = target_x - offset_Tx(i);
         R_Tx           = sqrt(element_x^2 + target_y^2);
         target_angle   = atan2(target_y,element_x);% expressed in radians
         
         % Get the Tx gain for this angle
         [~,index]      = min(abs((antenna_gain_pattern(:,1)) - target_angle));
         G_Tx_lin       = antenna_gain_pattern(index,2);
         
         %% process each Receiver
         
         for j = 1:N_Rx
            
            % calculate the range from Tx to the target
            % R_Rx1 = sqrt( (x - offset_Rx1)^2 + y^2 );
            element_x      = target_x - offset_Rx(j);
            
            R_Rx           = sqrt(element_x^2 + target_y^2);
            target_angle   = atan2(target_y,element_x);% expressed in radians
            
            % Get the Tx gain for this angle
            [~,index]      = min(abs((antenna_gain_pattern(:,1)) - target_angle));
            G_Rx_lin       = antenna_gain_pattern(index,2);
            
            %% What is the power return?
            
            % The power received at the Rx antenna terminals is given by
            % by the radar (power) equation:
            % Pr = (Pt_linear * G_Tx_linear * G_Rx_linear * lambda_linear^2 * RCS_linear) /
            % ((4*pi)^3 * R_Tx^2 * R_Rx^2)
            
            Pr    = (Pt_radar * G_Tx_lin * G_Rx_lin * lambda^2 * target_RCS) / ...
               ((4*pi)^3 * R_Tx^2 * R_Rx^2);
            
            %%  What is the delay for this target?
            
            % This is the time delay out and back from the target. Need to use
            % both the R_Tx and R_Rx to calculate the total delay time:
            % T_delay = (R_Tx + R_Rx) / c  [m / (m/s) = s]
            
            T_delay = (R_Tx + R_Rx) / c_light;
            
            %% Calculate the beat frequency
            
            % f_beat = alpha_chirp * T_delay;
            % But, alpha_chirp has the units of MHz/us
            % So, need to mulitiple alpha by: (10^6) / (10^(-6))
            
            f_beat = (alpha_chirp * (10^6) / (10^(-6))) * T_delay;

            %% Calculate the phase of the beat frequency
            
            % The phase is given by:
            phi_all = (2*pi*f_start*T_delay) + ...
               (pi*alpha_chirp*((10^6) / (10^(-6)))*T_delay^2);
            
            % The phase is some number between 0 and 2*pi.
            % Take the 2*pi modulus to find the discrete phase
            
            phi = mod(phi_all,2*pi);
            
            %% What are the I and Q voltage time-series during the chirp? after quadraturing mixing?
            
            ADC_I_voltage   = sqrt(Pr) .* cos((2*pi).*f_beat.*ADC_t_s + phi);
            ADC_Q_voltage   = sqrt(Pr) .* sin((2*pi).*f_beat.*ADC_t_s + phi);
            
            %% Add these voltages to this chirp sample
            
            ADC_I_voltage_TxRx(i,j,k,:)   = squeeze(ADC_I_voltage_TxRx(i,j,k,:)) + ADC_I_voltage';
            ADC_Q_voltage_TxRx(i,j,k,:)   = squeeze(ADC_Q_voltage_TxRx(i,j,k,:)) + ADC_Q_voltage';
            
            %% Add noise to each range gate and calculate SNR
            
            if(add_system_noise_flag == 1)
               
               [I_noise, Q_noise, ~]  = func_exponential_pwr_sample(Pn_linear,N_sample);
               
               %Pr_radar(k,:)  = Pr_radar(k,:) + IQ;
               ADC_I_voltage_TxRx(i,j,k,:)  =  squeeze(ADC_I_voltage_TxRx(i,j,k,:)) + I_noise';
               ADC_Q_voltage_TxRx(i,j,k,:)  =  squeeze(ADC_Q_voltage_TxRx(i,j,k,:)) + Q_noise';
               
            end % end if(add_system_noise_flag == 1)
                        
         end % end for j loop
         
      end % end for i loop
      
   end % end for e_target loop
   
end % end for k loop

%% Section IV. Process the ADC voltages
% =====================================

%% The data consists of ADC sample for each (Tx,Rx,N_chirp,ADC sample)

% The format is: (Tx, Rx, Chirp #, ADC sample)
% ADC_I_voltage_TxRx      2x4x64x256            1048576  double              
% ADC_Q_voltage_TxRx      2x4x64x256            1048576  double    

%% Perform the range_FFT for each chirp

% Define the range_FFT for each: (Tx, Rx, N_chirp, N_range)
%[m,n,p,q]   = size(ADC_I_voltage_TxRx);
%range_FFT   = zeros(m,n,p,q/2);
range_FFT   = zeros(N_Tx,N_Rx,N_chirp,N_range);

% Define the Hanning window for N_sample range_FFT
hann_range_window = hann(N_sample);

% Now Perform the range_FFT for each Chirp
for k = 1:N_chirp
   
   % for each Tx
   for i = 1:N_Tx
         
      %process each Receiver         
      for j = 1:N_Rx
      
         % get the ADC I & Q voltages
         I_input  = squeeze(ADC_I_voltage_TxRx(i,j,k,:));
         Q_input  = squeeze(ADC_Q_voltage_TxRx(i,j,k,:));

         % apply a Hanning window to the input voltages
         I_input  = hann_range_window .* I_input;
         Q_input  = hann_range_window .* Q_input;
         
         [IQ_fft_complex, ~] = func_calc_complex_FFT(I_input, Q_input);
         
         range_FFT(i,j,k,:)  = IQ_fft_complex(DC_index:N_sample);
         
      end % end for j loop
   end % end for i loop   
end % end for k loop

%% Perform the Doppler_FFT for each (Tx, Rx) pair

%[m,n,p,q]   = size(range_FFT);

range_Doppler_FFT    = ones(N_Tx,N_Rx,N_range,N_chirp) .* NaN;

% Define the Hanning window for N_chirp Doppler_FFT
hann_Doppler_window = hann(N_chirp);

% for each Tx
for i = 1:N_Tx
   
   %process each Receiver
   for j = 1:N_Rx
      
      for k = 1:N_range
         
         % get Re{range_FFT} and Im{range_FFT} voltages
         I_input  = real(squeeze(range_FFT(i,j,:,k)));
         Q_input  = imag(squeeze(range_FFT(i,j,:,k)));
      
         % apply a Hanning window to the input voltages
         I_input_win  = hann_Doppler_window .* I_input;
         Q_input_win  = hann_Doppler_window .* Q_input;
      
         [IQ_fft_complex, ~] = func_calc_complex_FFT(I_input_win, Q_input_win);
      
         range_Doppler_FFT(i,j,k,:)  = IQ_fft_complex;
         
      end % end for k loop
   end % end for j loop
end % end for i loop

%% Estimate composite range_Doppler power spectrum

% This section combines the power spectrum from all virtual antennas

composite_range_Doppler_FFT_pow  = ones(N_range,N_chirp);

for k = 1:N_range
   
   for e = 1:N_chirp
      
      complex_matrix    = abs(squeeze(range_Doppler_FFT(:,:,k,e)));
      f                 = ~isnan(complex_matrix);
      if(sum(f) > 0)
         composite_range_Doppler_FFT_pow(k,e)   = mean(complex_matrix(f));
      end % end if(sum(f) > 0)
      
   end % end for e loop
   
end % end for k loop

%% Find the noise threshold for each range gate

% process each range gate
nfft                 = 8;
initial_seed_length  = 8;

composite_noise_mean       = zeros(N_range,1);
composite_noise_threshold  = zeros(N_range,1);
composite_noise_std        = zeros(N_range,1);

for k = 1:N_range
   
   % get the power spectrum
   spk   = composite_range_Doppler_FFT_pow(k,:);
   
   % estimate the mean and max noise values
   [composite_noise_mean(k), composite_noise_threshold(k)] = func_find_mean_HS_noise(spk, nfft, initial_seed_length);
   
   % estimate the std(noise)
   f  = spk < composite_noise_threshold(k);
   if(sum(f) > 2)
      composite_noise_std(k)  = std(spk(f));
   end % end if(sum(f) > 2)
      
end % end for k loop

%% Plot the composite range_Doppler power spectra

% % The velocity bins are defined with Vd
% % the range gates are defined with: distance_each_range_gate
%
% d_distance  = distance_each_range_gate(6) - distance_each_range_gate(5);
% dVd         = Vd(6) - Vd(5);
%
% plot_spc   = 10.*log10(composite_range_Doppler_FFT_pow);
%
% figure
% colormap('jet')
%
% subplot(2,1,1)
% pcolor(Vd-dVd/2,distance_each_range_gate - d_distance/2, plot_spc);
% shading flat
% colorbar
% xlabel('Doppler Velocity [m/s]')
% ylabel('Range [m]')
% title('Composite Range-Doppler Power Spectra [dB]')
%
% % remove all data below the noise_threshold
% plot_spc_threshold    = composite_range_Doppler_FFT_pow;
% for k = 1:N_range
%    f = plot_spc_threshold(k,:) < composite_noise_threshold(k);
%    if(sum(f) > 0)
%       plot_spc_threshold(k,f) = plot_spc_threshold(k,f) .* NaN;
%    end % end if(sum(f) > 0)
% end % end for k loop
%
% plot_spc_threshold   = 10.*log10(plot_spc_threshold);
%
% subplot(2,1,2)
% pcolor(Vd-dVd/2,distance_each_range_gate - d_distance/2, plot_spc_threshold);
% shading flat
% colorbar
% xlabel('Doppler Velocity [m/s]')
% ylabel('Range [m]')
% title('Composite Range-Doppler Power Spectra above Noise Threshold [dB]')

%% Find all 2D peaks in composite range-Doppler image

% The logic is to find largest magnitude peak, then find integration limits
% in Doppler and range space. 

% rename the composite range-Doppler powe spectra
% This matrix will be destroyed while finding the peaks.
spk_peak_removed   = composite_range_Doppler_FFT_pow;

% Set all values below noise threshold to NaN.
for k = 1:N_range
   f = spk_peak_removed(k,:) < composite_noise_threshold(k);
   if(sum(f) > 0)
      spk_peak_removed(k,f) = spk_peak_removed(k,f) .* NaN;
   end % end if(sum(f) > 0)
end % end for k loop

% Define the number of targets to find.
max_num_targets   = 64;

% define the indices:
% col    index
%  1      left_first_column_index
%  2      right_first_column_index
%  3      closer_second_column_index
%  4      further_second_column_index
target_indices       = ones(max_num_targets,4) * NaN;
target_num_indices   = ones(max_num_targets,1) * NaN;

get_more_targets  = 1;
total_num_targets = 0;

while(get_more_targets)
   
   % find the largest magnitude peak   
   [spk_peak_removed, index_limit, num_indices] = func_find_largest_2D_peak(spk_peak_removed);
   
   if(num_indices >= 1)
      total_num_targets                         = total_num_targets + 1;
      target_indices(total_num_targets,:)       = index_limit;
      target_num_indices(total_num_targets,:)   = num_indices;
      
   else
      get_more_targets  = 0;
   end % end if(num_indices > 1)
   
   % safety value, limit the number of targets
   if(total_num_targets >= max_num_targets)
      get_more_targets = 0;
   end % end if(total_num_targets >= max_num_targets)
   
end % end while(get_more_targets)

%% For each identified target, find the angle of arrival

% Need to use the complex values stored in the range_Doppler_FFT:
% range_Doppler_FFT(i,j,k,:)  = IQ_fft_complex;

target_range      = ones(1,total_num_targets) .* NaN;
target_angle      = ones(1,total_num_targets) .* NaN;
target_velocity   = ones(1,total_num_targets) .* NaN;
target_magnitude  = ones(1,total_num_targets) .* NaN;
target_x          = ones(1,total_num_targets) .* NaN;
target_y          = ones(1,total_num_targets) .* NaN;
target_snr        = ones(1,total_num_targets) .* NaN;

% Define the output radar_target_list
% One target per row.
% columns are defined as:
% column    Description
%   1       SNR - signal to noise ratio
%   2       x distance (postive to the right of radar)
%   3       y distance 
%   4       z distance (positive is above radar)
%   5       Doppler velocity (positive = target approaching radar)
radar_target_list    = ones(total_num_targets,5) .* NaN;

for e = 1:total_num_targets
   
   % find the angle-of-arrival for each range-Doppler pixel of target
   
   target_angle_temp       = zeros(target_num_indices(e),1);
   target_range_temp       = zeros(target_num_indices(e),1);
   target_velocity_temp    = zeros(target_num_indices(e),1);
   target_magnitude_temp   = zeros(target_num_indices(e),1);
   target_power_temp       = zeros(target_num_indices(e),1);
   pixel_index             = 0;
   
   % process each pixel of target
   for r = target_indices(e,1):target_indices(e,2)
      for c = target_indices(e,3):target_indices(e,4)
         
         % number this pixel
         pixel_index                      = pixel_index + 1;
         % pre-define the virtual antenna values
         virtual_antenna_complex_value    = ones(8,1) .* NaN;
         virtual_antenna_range            = ones(8,1) .* NaN;
         virtual_antenna_velocity         = ones(8,1) .* NaN;

         index                            = 0;
         
         % for each Tx
         for i = 1:N_Tx
            
            % process each Receiver
            for j = 1:N_Rx
               
               % get the complex values
               index                                  = index + 1;
               virtual_antenna_complex_value(index)   = range_Doppler_FFT(i,j,r,c);
               virtual_antenna_range(index)           = distance_each_range_gate(r);
               virtual_antenna_velocity(index)        = Vd(c);
               
            end % end for c loop
         end % end for r loop
                  
         % At this point, have virtual antenna complex values
         % Calculate the angle-FFT
         % process each column
         angle_Npts                       = 64;
         
         % Get the real and imaginary parts
         I_input  = real(virtual_antenna_complex_value);
         Q_input  = imag(virtual_antenna_complex_value);
         
         [angle_FFT_power, angle_of_arrival_deg] = func_calc_angle_FFT(I_input, Q_input, angle_Npts);
         
         % get the larget magnitude angle
         [~, index]                          = max(angle_FFT_power);
         target_angle_temp(pixel_index)      = angle_of_arrival_deg(index);
         target_magnitude_temp(pixel_index)  = angle_FFT_power(index);
         target_power_temp(pixel_index)      = sum(sqrt(I_input.^2 + Q_input.^2));
         
         % get the range to the target
         target_range_temp(pixel_index)      = mean(virtual_antenna_range);
         target_velocity_temp(pixel_index)   = mean(virtual_antenna_velocity);
         
         %angle_FFT_Npts_power(:,c)   = angle_FFT_power;
      end % end for j loop
      
   end % end for i loop
   
   % Find the weighted average values
   wt_sum               = sum(target_magnitude_temp);
   range_wt             = target_range_temp .* target_magnitude_temp;
   angle_wt             = target_angle_temp .* target_magnitude_temp;
   velocity_wt          = target_velocity_temp .* target_magnitude_temp;
   
   target_range(e)      = sum(range_wt) / wt_sum;
   target_angle(e)      = sum(angle_wt) / wt_sum;
   target_velocity(e)   = sum(velocity_wt) / wt_sum;
   target_magnitude(e)  = mean(target_magnitude_temp);
   
   % Convert the (range,angle) to (x,y)
   target_x(e)          = target_range(e) * sin(target_angle(e) .*(pi/180));
   target_y(e)          = target_range(e) * cos(target_angle(e) .*(pi/180));
   
   % calculate the SNR for this target
   % get the range gate index for this target range
   [~, range_index] = min(abs(target_range(e) - distance_each_range_gate));
   % get the noise power 
   noise_pow   = composite_noise_mean(range_index) .* length(Vd);
   % get the signal power
   signal_power   = sum(target_power_temp);
   % calculate the SNR
   if((signal_power > 0) && (noise_pow > 0))       %#ok<BDSCI>
      target_snr(e)  = 10.*log10(signal_power / noise_pow);
   end % end if(valid signal and noise powers)

   % populate the output target list:
   % column    Description
   %   1       SNR - signal to noise ratio
   %   2       x distance (postive to the right of radar)
   %   3       y distance 
   %   4       z distance (positive is above radar)
   %   5       Doppler velocity (positive = target approaching radar)
   radar_target_list(e,1)  = target_snr(e);
   radar_target_list(e,2)  = target_x(e);
   radar_target_list(e,3)  = target_y(e);
   radar_target_list(e,4)  = 0;
   radar_target_list(e,5)  = target_velocity(e);
  
end % end for e loop

%% Plot the range-Doppler and x-y plots

% % The velocity bins are defined with Vd
% % the range gates are defined with: distance_each_range_gate
%
% d_distance  = distance_each_range_gate(6) - distance_each_range_gate(5);
% dVd         = Vd(6) - Vd(5);
%
% %plot_spc   = 10.*log10(composite_range_Doppler_FFT_pow);
%
% figure
% colormap('jet')
%
% % remove all data below the noise_threshold
% plot_spc_threshold    = composite_range_Doppler_FFT_pow;
% for k = 1:N_range
%    f = plot_spc_threshold(k,:) < composite_noise_threshold(k);
%    if(sum(f) > 0)
%       plot_spc_threshold(k,f) = plot_spc_threshold(k,f) .* NaN;
%    end % end if(sum(f) > 0)
% end % end for k loop
%
% plot_spc_threshold   = 10.*log10(plot_spc_threshold);
%
% subplot(2,1,1)
% pcolor(Vd-dVd/2,distance_each_range_gate - d_distance/2, plot_spc_threshold);
% shading flat
% colorbar
% hold on
% for i = 1:total_num_targets
%    if(~isnan(target_num_indices(i)))
%       x_indices   = [target_indices(i,3)-1 target_indices(i,4)+1 target_indices(i,4)+1 target_indices(i,3)-1 target_indices(i,3)-1];
%       % check the boundaries of the x_indices
%       f  = x_indices < 1;
%       if(sum(f) > 0)
%          x_indices(f) = ones(sum(f),1);
%       end % end if((sum(f) > 0)
%       f  = x_indices > length(Vd);
%       if(sum(f) > 0)
%          x_indices(f) = ones(sum(f),1)*length(Vd);
%       end % end if((sum(f) > 0)
%       x_data      = Vd(x_indices);
%
%       y_indices   = [target_indices(i,1)-1 target_indices(i,1)-1 target_indices(i,2)+1 target_indices(i,2)+1 target_indices(i,1)-1];
%       % check the boundaries of the x_indices
%       f  = y_indices < 1;
%       if(sum(f) > 0)
%          y_indices(f) = ones(sum(f),1);
%       end % end if((sum(f) > 0)
%       f  = y_indices > length(distance_each_range_gate);
%       if(sum(f) > 0)
%          y_indices(f) = ones(sum(f),1)*length(distance_each_range_gate);
%       end % end if((sum(f) > 0)
%
%       y_data      = distance_each_range_gate(y_indices);
%       plot(x_data,y_data,'r','linewidth',0.5)
%    end % end if(~isnan(target_num_indices(i)))
% end % end for i loop
% axis([-10 10 0 5.5])
% grid on
% set(gca,'xtick',-10:1:10,'xticklabel',{'-10',' ','-8',' ','-6',' ','-4',' ','-2',' ','0',' ','2',' ','4',' ','6',' ','8',' ','10'});
% set(gca,'ytick',0:0.5:5);
% xlabel('Doppler Velocity [m/s]')
% ylabel('Range [m]')
% title('a. Composite Range-Doppler Power Spectra [dB]')
%
% subplot(2,1,2)
% scatter(target_x,target_y,10,target_snr,'filled')
% hold on
% plot([0 5],[0 5],'k','linewidth',1)
% plot([0 -5],[0 5],'k','linewidth',1)
% shading flat
% colorbar
% caxis([-10 20])
% hold on
% grid on
% axis([-5 5 0 5.5])
% set(gca,'xtick',-5:0.5:5,'xticklabel',{'-5',' ','-4',' ','-3',' ','-2',' ','-1',' ','0',' ','1',' ','2',' ','3',' ','4',' ','5'});
% set(gca,'ytick',0:0.5:5);
% %set(gca,'xtick',start_hour:1:end_hour,'xticklabel',{'0',' ',' ','3',' ',' ','6',' ',' ','9',' ',' ','12',' ',' ','15',' ',' ','18',' ',' ','21',' ',' ','24'});
%
% xlabel('x-Distance [m]')
% ylabel('y-Distance [m]')
% title('b. Target (x,y) Estimates')

