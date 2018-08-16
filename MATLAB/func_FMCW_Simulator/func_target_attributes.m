function [target_attribute] = func_target_attributes

% This routine defines targets for the fmcw simulator.

%% Define 2D matrix of target attributes

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

%% Define the number of targets

N_target    = 4;

% Define the target 2D matrix (which will be sent to radar simulator)
target_attribute    = zeros(N_target,5);

%% Define target size (radar cross section RCS) is in m^2.

% (all the same size...)
target_attribute(:,1)     =  ones(N_target,1) .* 0.1;

%% Define initial (x,y) location

% The target is defined in the (x,y) plane in front of the radar
% Radar is located at (x,y) = (0,0);
% y-axis is perpendicular to the radar (straight in front of radar), y >= 0;
% x-axis is normal to the radar bore sight. 
% x < 0 is to the left of the radar, 
% x > 0 is the right of the radar.
target_attribute(:,2)     =  [ 0    0  0  0];
target_attribute(:,3)     =  [ 0.5  2  2  3];

%% Define the target speed and direction of motion

% Target speed is in m/s. (Speed is always positive)
target_attribute(:,4)     =  [ 0  1  1  1];

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
target_attribute(:,5)     =  [ 0  90 270 180];
