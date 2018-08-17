function [target_attributes] = func_move_targets(target_attributes, sim_delta_time)

% This function moves the targets to new positions

% Inputs
% target_attributes     - % Each row corresponds to a different target.
%                          Each column corresponds to a different target attribute.
%                          Matrix size is (N_targets,5) 
%                           where:
%                          N_targets    - number of targets
%                           column    Description
%                             1       target size (radar cross section RCS) is in m^2.
%                             2       x position [m/s]
%                             3       y position [m/s]
%                             4       target speed [m/s]
%                             5       target attack angle [degrees]
% sim_delta_time        - time between frames

% Outputs:
% target_attributes     - updated target_attributes (see above)

% updated: 16-August-2018
% =========================================================================

%% Get the number of targets

[num_targets,~]    = size(target_attributes);

%% Loop through each target and update its position

for e = 1:num_targets
   
   %% Define the new target location
   
   % Don't need to keep the old location of the target
   % Just move the target to a new position
   
   % calculate the target gradient in x and y directions
   target_mx    = cos((pi/180) .* target_attributes(e,5));
   target_my    = sin((pi/180) .* target_attributes(e,5));
   
   % update the target's location:
   % x_new  = x_old + mx*(speed*dT);
   % y_new  = y_old + my*(speed*dT);
   
   target_x  = target_attributes(e,2) + target_mx * (target_attributes(e,4) * sim_delta_time);
   target_y  = target_attributes(e,3) + target_my * (target_attributes(e,4) * sim_delta_time);
   
   % save these new locations for the next chirp
   target_attributes(e,2)  = target_x;
   target_attributes(e,3)  = target_y;
   
end % end for e loop


