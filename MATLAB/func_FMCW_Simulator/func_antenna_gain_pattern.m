function [antenna_gain_pattern] = func_antenna_gain_pattern

% This function generates the antenna gain pattern used by the antennas.

% Inputs
% all inputs are loaded via a saved matlab mat file

% Outputs:
% antenna_gain_pattern - Lookup table of angle and antenna gain.
%                          The two columns are:
%                          col     Description
%                           1      angle in degrees, 180 = left, 90 = straight in front, 0 = right
%                           2      antenna gain, in linear units.

% updated: 16-August-2018
% ========================================================================

%% Load the antenna pattern

% load an antenna pattern
load('antenna_pattern_N02.mat','antenna_gain_pattern_linear','theta_range')
% the variables are:
%  antenna_efficiency                   1x1                 8  double              
%  antenna_gain_pattern_linear       1001x1              8008  double              
%  normalized_radiation_pattern      1001x1              8008  double              
%  theta_range                       1001x1              8008  double              

% theta_range goes from -pi/2 to pi/2 with 0 = boresight
% theta = -pi/2 is to the left, theta = pi/2 is to the right 

% Convert the angle to be math coordinates
% want angle to go from 180 (left), 90 (straight up), 0 (right)
angle_math               = (theta_range - pi/2) *(-1);

% Generate a lookup table for the antenna gain.
% The two columns are:
% col     Description
%  1      angle in degrees, 180 = left, 90 = straight in front, 0 = right
%  2      antenna gain, in linear units.
antenna_gain_pattern(:,1)  = angle_math;
antenna_gain_pattern(:,2)  = antenna_gain_pattern_linear;
