function [right_target_FFT_mag, right_target_FFT_ang, right_null_FFT_mag, right_null_FFT_ang] = ...
   func_find_right_target(max_value, max_index, short_mag_array, short_ang_array,...
   max_num_angles)

% Inputs
% max_value       - maximum magnitude
% max_index       - index at maximum magnitude
% short_mag_array - magnitude of array
% short_ang_array - angles of array
% max_num_angles  - length of array

% Outputs
% right_null_FFT_mag  - magnitude of local min
% right_null_FFT_ang  - angle of local min

% updated: 15-Oct-2018
% ======================================================================

% Look to the right
% -----------------
current_index  = max_index;
current_value  = max_value;
next_index     = current_index;
get_next_flag  = 1;

while(get_next_flag)
   % decrement the index
   next_index = next_index + 1;
   
   % at end of array?
   if(next_index > max_num_angles)
      right_null_index  = max_num_angles;
      get_next_flag  = 0;
   else
      % If got to here, then still within array. Process it.
      
      % get the value value in the array
      next_value  = short_mag_array(next_index);
      
      % Is the next_value less than the current_value?
      if(next_value < current_value)
         current_value = next_value;
         current_index = next_index;
      else
         % Get here means that current_index is pointing to null.
         right_null_index  = current_index;
         get_next_flag     = 0;
      end % end if(next_value < current_value)
      
   end % end if(next_index > max_num_angles)
   
end % end while(get_next_flag)

% save the right null values
right_null_FFT_mag  = short_mag_array(right_null_index);
right_null_FFT_ang  = short_ang_array(right_null_index);

% Find the largest peak to the right
% ==================================
% mag_array has all values between -46 and +46.
% What to search a subset of this range
mag_subset  = short_mag_array(right_null_index:max_num_angles);
ang_subset  = short_ang_array(right_null_index:max_num_angles);

[max_value, max_index] = max(mag_subset);
right_target_FFT_mag  = max_value;
right_target_FFT_ang  = ang_subset(max_index);

      