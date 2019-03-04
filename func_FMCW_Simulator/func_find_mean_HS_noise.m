function [mean_noise, max_noise] = func_find_mean_HS_noise(spk, nfft, initial_seed_length)

% Estimate the mean and maximum noise values from H-S method.

% Inputs:
% spk                   - single spectrum in power.  Linear and positive.
% nfft                  - number of averaged ffts
% initial_seed_length   - number of points used to develop intial value
%
% Output:
% mean_noise    - Mean of all values in noise.
% max_noise     - Maximum value of noise values.
%
% This code adapted from Tony Riddle's NOAA Aeronomy Lab code.
% Updated: 26-March-2014
% ========================================================================

%% Remove negative spectrum values

% Sometimes, the spectrum contains bad values that are negative.
% If negative values exist, then shift spectrum so that min value is zero.
min_spk = min(spk);
if(min_spk < 0)
    spk = spk + abs(min_spk);
end   % end if(min_spk < 0)

%% Sort the spectrum into accending order.

sort_spk = sort(spk);

%% Set some initial values

rtest = (1 + nfft)/nfft;
xsum     = 0;
xsum_sq  = 0;

%% pre-load the xsum and xsum_sq values from inital_seed_length

% Processes the first few samples without testing for any signal
n_load = initial_seed_length;
for i = 1:n_load
   xsum    = xsum    + sort_spk(i);
   xsum_sq = xsum_sq + sort_spk(i)*sort_spk(i);
end % end for i loop

%% Process values past the initial_seed_length

% point to the next value
i = n_load + 1;

noise_index = length(spk);
get_more    = 1;

while(get_more)
   
   xsum    = xsum    + sort_spk(i);
   xsum_sq = xsum_sq + sort_spk(i)*sort_spk(i);
   
   if(xsum_sq*i > rtest*xsum*xsum)
      noise_index = i-1;
      get_more = 0;
      if(noise_index < 1)
         noise_index = 1;
      end % end if(noise_index < 1)      
   end % end if(xsum_sq*i <= rtest*xsum*xsum)

   %disp(['i: ',num2str(i),', noise_index: ',num2str(noise_index)]);
   
   i = i + 1;
   if(i > noise_index)
      get_more = 0;
   end % end if(i > last_i)
   
end % end while(get_more) 

%% Save the mean and max noise values for output

% Remember to shift the noise values because the spectrum was shifted.
if(min_spk < 0)
    mean_noise = mean(sort_spk(1:noise_index)) - abs(min_spk);
    max_noise  = sort_spk(noise_index) - abs(min_spk);
else
    mean_noise = mean(sort_spk(1:noise_index));
    max_noise  = sort_spk(noise_index);
end % end if(min_spk < 0)
    