function [spk_peak_removed, index_limit, num_indices] = func_find_largest_2D_peak(spk)

% Find a single peak containing the largest magnitude spectrum value.

% Input Variables
% spk             - input spectrum, bad values contain NaN

% Output Variables
% spk_clean     the same as input spk except outside points are NaN'd
% index_limit   End index values for the found peak
%               index_limit(1) -first velocity bin used in peak
%               index_limit(2) -last veocity bin used in peak
% Note: spk(index_limit(1):index_limit(2)) are all valid spectrum values 

% This routine finds the single peak by finding the maximum value and
%   marching down both sides of the peak until a NaN is found.

% Steps:
% 1. Check to see that there are valid points in spectra
% 2. Find the maximum value in the spectra.
% 3. Move to the left of the max value
% 4. Move to the left of the max value
% 5. NaN out all values outside of the indices

% Updated: 27-June-2018
% ************************************************************************

%% 1. Check to see that there are valid points in spectra

% There needs to be at least 2 points above the noise
f = ~isnan(spk);

if(sum(f) < 2)
    spk_peak_removed = spk;
    index_limit      = [NaN NaN NaN NaN];
    return
end % end if(sum(f) < 2)

%% 2. Find the location of the maximum value in the matrix.

r_max_index    = 0;
c_max_index    = 0;
max_value    = 0;


[m,n]    = size(spk);

for r = 1:m

   [test_value, test_index] = max(spk(r,:));
   if(test_value > max_value)
      max_value   = test_value;
      c_max_index = test_index;
      r_max_index = r;
   end % end if(test_value > c_max_value)
   
end % end for r loop

% index: (r_max_index,c_max_index) points to the largest magnitude value
      

%% 3. First Column (r_index): Move to the left (smaller values) of the max value

% This section moves along the first column
current_r_index   = r_max_index;
current_c_index   = c_max_index;

next_r_index      = current_r_index;
get_next          = 1;

while(get_next)
    % decrement the index
    next_r_index = next_r_index - 1;
    
    if(next_r_index < 1)
        next_r_index    = 1;
        get_next      = 0;
    end % end if(next_index < 1)
    
    % Get the value of the next spectral point
    next_value = spk(next_r_index, current_c_index);
    
    % Is next_value below the noise threshold?
    if(isnan(next_value))
        get_next = 0;
        % current_r_index is back one value...
    
    else  % the next value is value, is it decreasing or increasing?

       % move the current_index       
       current_r_index = next_r_index;
    
    end % end if(isnan(next_value))
    
end % end while(get_next)

% Place the current_index into the output variable
index_limit(1) = current_r_index;

%% 4. First Column (r_index): Move to the right (larger values) of the max value

current_r_index   = r_max_index;
current_c_index   = c_max_index;

next_r_index      = current_r_index;
get_next          = 1;

while(get_next)
    % increment the index
    next_r_index = next_r_index + 1;
    
    if(next_r_index > m)
        next_r_index    = m;  % dimension of 1st column
        get_next      = 0;
    end % end if(next_index > length(spk))
    
    % Get the value of the next spectral point
    next_value = spk(next_r_index,current_c_index);
    
    % Is next_value below the noise threshold?
    if(isnan(next_value))
        get_next = 0;
        % current_index is back one value
        
    else
       
       % move the current_index 
       current_r_index = next_r_index;
    
    end % end if(isnan(next_value))

end % end while(get_next)

% Place the current_index into the output variable
index_limit(2) = current_r_index;

%% 5. Second Column (c_index): Move to the left (smaller values) of the max value

% This section moves along the first column
current_r_index   = r_max_index;
current_c_index   = c_max_index;

next_c_index      = current_c_index;
get_next          = 1;

while(get_next)
    % decrement the index
    next_c_index = next_c_index - 1;
    
    if(next_c_index < 1)
        next_c_index    = 1;
        get_next      = 0;
    end % end if(next_index < 1)
    
    % Get the value of the next spectral point
    next_value = spk(current_r_index,next_c_index);
    
    % Is next_value below the noise threshold?
    if(isnan(next_value))
        get_next = 0;
        % current_r_index is back one value...
    
    else  % the next value is value, is it decreasing or increasing?

       % move the current_index       
       current_c_index = next_c_index;
    
    end % end if(isnan(next_value))
    
end % end while(get_next)

% Place the current_index into the output variable
index_limit(3) = current_c_index;

%% 6. Second Column (c_index): Move to the right (larger values) of the max value

current_r_index   = r_max_index;
current_c_index   = c_max_index;

next_c_index      = current_c_index;
get_next          = 1;

while(get_next)
    % increment the index
    next_c_index = next_c_index + 1;
    
    if(next_c_index > n)
        next_c_index    = n;  % dimension of 2nd column
        get_next      = 0;
    end % end if(next_index > length(spk))
    
    % Get the value of the next spectral point
    next_value = spk(current_r_index,next_c_index);
    
    % Is next_value below the noise threshold?
    if(isnan(next_value))
        get_next = 0;
        % current_index is back one value
        
    else
       
       % move the current_index 
       current_c_index = next_c_index;
    
    end % end if(isnan(next_value))

end % end while(get_next)

% Place the current_index into the output variable
index_limit(4) = current_c_index;


%% 7. NaN out all values inside of the indices

spk_peak_removed = spk;

% NaN out all values within 2D box defined by index_limit

for r = index_limit(1):index_limit(2)
   for c = index_limit(3):index_limit(4)
      spk_peak_removed(r,c)   = NaN;
   end % end for c loop
end % end for r loop

%% 8. Count the number of indices found with peak

num_r_indices  = index_limit(2) - index_limit(1) + 1;
num_c_indices  = index_limit(4) - index_limit(3) + 1;

num_indices    = num_r_indices .* num_c_indices;
