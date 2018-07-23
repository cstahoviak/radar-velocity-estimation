% program: main_fmcw_radar_simulator_2018_0721.m
% updated: 21-July-2018

% This main routine enables the FMCW simulator to be function call.

% The user must define the targets.

% In this version, all radar parameters are defined within the funtion
% call. In the future, to improve speed, some of these parameters will be 
% pre-defined and inputs to the radar simulator function.


%% Section 1. Define the radar target attributes
% ========================================================================

% For each target, need to define:
%     target size (radar cross section (RCS))
%     target x position
%     target y position
%     target speed (m/s)
%     target direction of travel (attack angle, degrees)

% Input for the simulator:
% =======================
% Define a 2-D matrix of target attributes. 
% Each row corresponds to a different target.
% Each column corresponds to a different target attribute.
% Matrix size is (N_targets,5) 
%  where:
% N_targets    - number of targets
% column    Description
%   1       target size (radar cross section RCS) is in m^2.
%   2       x position [m/s]
%   3       y position [m/s]
%   4       target speed [m/s]
%   5       target attack angle [degrees]

% The attributes are defined as:
% The target size (radar cross section RCS) is in m^2.

% The target is defined in the (x,y) plane in front of the radar
% Radar is located at (x,y) = (0,0);
% y-axis is perpendicular to the radar (straight in front of radar), y >= 0;
% x-axis is normal to the radar bore sight. 
% x < 0 is to the left of the radar, 
% x > 0 is the right of the radar.

% Target speed is in m/s.

% The target direction of travel is defined by the attack angle and follows
% the angle of the complex plane: 
% attack angle =   0, moving along +x direction, motion from left to right, 
%                       (x increasing, y not changing) 
% attack angle =  90, moving in the +y direction, motion away from radar
%                       (x not changing, y increasing)
% attack angle = 180, moving in the -x direction, motion from right to left
%                       (y not changing, x decreasing)
% attack angle = 270, moving in the -y direction, motion toward the radar
%                       (x not changing, y decreasing)

%%    Define the target attributes
% ===============================================

% Define a 2-D matrix of target attributes. 
% Each row corresponds to a different target.
% Each column corresponds to a different target attribute.
% Matrix size is (N_targets,5) 
%  where:
% N_targets    - number of targets
% column    Description
%   1       target size (radar cross section RCS) is in m^2.
%   2       x position [m/s]
%   3       y position [m/s]
%   4       target speed [m/s]
%   5       target attack angle [degrees]

% Define the number of targets
N_target    = 4;

% Define the target 2D matrix (which will be sent to radar simulator)
target_attribute    = zeros(N_target,5);

% Define target size (radar cross section RCS) is in m^2.
% (all the same size...)
target_attribute(:,1)     =  ones(N_target,1) .* 0.1;

% The target is defined in the (x,y) plane in front of the radar
% Radar is located at (x,y) = (0,0);
% y-axis is perpendicular to the radar (straight in front of radar), y >= 0;
% x-axis is normal to the radar bore sight. 
% x < 0 is to the left of the radar, 
% x > 0 is the right of the radar.
target_attribute(:,2)     =  [ 0  1 -2  2];
target_attribute(:,3)     =  [ 1  2  3  4];

% Target speed is in m/s. (Speed is always positive)
target_attribute(:,4)     =  [ 0  0.5 1  1];

% The target direction of travel is defined by the attack angle and follows
% the angle of the complex plane: 
% attack angle =   0, moving along +x direction, motion from left to right, 
%                       (x increasing, y not changing) 
% attack angle =  90, moving in the +y direction, motion away from radar
%                       (x not changing, y increasing)
% attack angle = 180, moving in the -x direction, motion from right to left
%                       (y not changing, x decreasing)
% attack angle = 270, moving in the -y direction, motion toward the radar
%                       (x not changing, y decreasing)
target_attribute(:,5)     =  [ 0  45 90 135];

%%    Section II. Interate through time moving targets with each interation

%% Define how often you want radar estimates.

% define the time between radar estimates
sim_delta_time    = 0.1;   % expressed in seconds

% Define either simulation duration or number of samples...
% define the simulation duration
sim_duration      = 1;  % expressed in seconds

% Determine the number of iterations
sim_N_interations    = round(sim_duration / sim_delta_time) + 1;

%% run through the interations....

for k = 1:sim_N_interations
   
   %% Call the radar simulator
   
   [radar_target_list]  = func_fmcw_radar_simulator_2018_0721(target_attribute);

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

   %% display a couple targets
   
   disp(' ')
   disp(['simulation run: ',num2str(k)]);
   disp('A couple targets from the Radar target list:')   
   N_list   = 8;
   radar_target_list(1:N_list,:)
   disp('     SNR       x [m]     y [m]     z [m]  Radial Velocity [m/s]')
      
         
   %% Move the target to their new locations
   
   [num_targets,~]    = size(target_attribute);
   
   for e = 1:num_targets
      
      %% Define the new target location
      
      % Don't need to keep the old location of the target
      % Just move the target to a new position
      
      % calculate the target gradient in x and y directions
      target_mx    = cos((pi/180) .* target_attribute(e,5));
      target_my    = sin((pi/180) .* target_attribute(e,5));
      
      % update the target's location:
      % x_new  = x_old + mx*(speed*dT);
      % y_new  = y_old + my*(speed*dT);
      
      target_x  = target_attribute(e,2) + target_mx * (target_attribute(e,4) * sim_delta_time);
      target_y  = target_attribute(e,3) + target_my * (target_attribute(e,4) * sim_delta_time);
      
      % save these new locations for the next chirp
      target_attribute(e,2)  = target_x;
      target_attribute(e,3)  = target_y;

   end % end for e loop
   
end % end for k loop

