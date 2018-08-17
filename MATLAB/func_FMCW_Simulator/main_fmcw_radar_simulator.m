% program: main_fmcw_radar_simulator.m
% updated: 16-August-2018

% This main routine enables the FMCW simulator to be function call.

% The user must define the targets.

%% Section I. Define the radar target attributes
% ========================================================================

% See function call for target attributes
[target_attributes] = func_target_attributes;

%%    Section II. Define Radar Attributes and Operating Parameters
% ========================================================================

%% Get the antenna pattern

[antenna_gain_pattern] = func_antenna_gain_pattern;

%% Define the Radar Operating Parameters

% This function call sets radar operating parameters for different user
% defined input strings. Valid strings are:
% 'best_range_res'   = best range resolution 

[chirp_parameters]   = func_defineChirpParameters( 'best_range_res' );

%%    Section III. Interate through time moving targets with each interation

%% Define how often you want radar estimates.

% define the time between radar estimates

%sim_delta_time    = 0.1;   % expressed in seconds
sim_delta_time    = chirp_parameters(6);   % expressed in seconds

% Define either simulation duration or number of samples...
% define the simulation duration
sim_duration      = 1;  % expressed in seconds

% Determine the number of iterations
%sim_N_interations    = round(sim_duration / sim_delta_time) + 1;

% Set sim_N_interations = 1 to transmit one Frame
sim_N_interations    = 1;

%% run through the interations....

for k = 1:sim_N_interations
   
   %% Call the radar simulator
   
   [radar_target_list, range_angle_heatmap_zeroVd, ...
   range_angle_range, range_angle_deg]  = ...
   func_fmcw_radar_simulator(target_attributes, ...
   antenna_gain_pattern, chirp_parameters);

   % output is:
   % radar_target_list  -  List of 64 targets
   %                       One target per row.
   %                       columns are defined as:
   %                       column    Description
   %                         1       SNR - signal to noise ratio
   %                         2       x distance (postive to the right of radar)
   %                         3       y distance
   %                         4       z distance (positive is above radar)
   %                         5       Doppler velocity (positive = target approaching radar)

   %% Plot the range_angle_heatmap and radar_target_list
      
   % The velocity bins are defined with Vd
   % the range gates are defined with: distance_each_range_gate
   
   d_distance  = range_angle_range(6) - range_angle_range(5);
   d_angle     = range_angle_deg(6) - range_angle_deg(5);
   
   %plot_spc   = 10.*log10(composite_range_Doppler_FFT_pow);
   
   figure
   colormap('jet')
   
   subplot(2,1,1)
   pcolor(range_angle_deg - d_angle/2,range_angle_range - d_distance/2,10.*log10(range_angle_heatmap_zeroVd));
   hold on
   shading flat
   colorbar
   %caxis([-100 -70])
   xlabel('Angle [deg]')
   ylabel('Range [m]')
   title('b. Range-Angle Heatmap, Zero Doppler [dB]')
   
   axis([-40 40 0 5.5])
   grid on
   set(gca,'xtick',-40:5:40,'xticklabel',{'-40',' ','-30',' ','-20',' ','-10',' ','0',' ','10',' ','20',' ','30',' ','40'});
   set(gca,'ytick',0:0.5:5);
   xlabel('Angle [degree]')
   ylabel('Range [m]')
   title('a. Range-Angle Heat Map at zero velocity [dB]')
   
   subplot(2,1,2)
   scatter(radar_target_list(:,2),radar_target_list(:,3),10,radar_target_list(:,1),'filled')
   hold on
   plot([0 5],[0 5],'k','linewidth',1)
   plot([0 -5],[0 5],'k','linewidth',1)
   shading flat
   colorbar
   caxis([-10 20])
   hold on
   grid on
   axis([-5 5 0 5.5])
   set(gca,'xtick',-5:0.5:5,'xticklabel',{'-5',' ','-4',' ','-3',' ','-2',' ','-1',' ','0',' ','1',' ','2',' ','3',' ','4',' ','5'});
   set(gca,'ytick',0:0.5:5);
   %set(gca,'xtick',start_hour:1:end_hour,'xticklabel',{'0',' ',' ','3',' ',' ','6',' ',' ','9',' ',' ','12',' ',' ','15',' ',' ','18',' ',' ','21',' ',' ','24'});
   
   xlabel('x-Distance [m]')
   ylabel('y-Distance [m]')
   title('b. Target (x,y) Estimates')
   
   
   %% display a couple targets
   
   disp(' ')
   disp(['simulation run: ',num2str(k-1),', time: ',num2str((k-1)*sim_delta_time),' seconds']);
   disp('A couple targets from the Radar target list:')   
   N_list   = 8;
   disp('     SNR       x [m]     y [m]     z [m]  Radial Velocity [m/s]')
   radar_target_list(1:N_list,:)
   
   %% Move the target to their new locations
   
   [target_attributes] = func_move_targets(target_attributes, sim_delta_time);
   
end % end for k loop
