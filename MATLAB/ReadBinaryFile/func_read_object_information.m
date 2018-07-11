function [numDetectObj, Q_factor, range_index, Doppler_index, peak_value, ...
   x_value, y_value, z_value, eof_flag] = func_read_object_information(fid)

% updated: 03-July-2018
% =======================================================================

%% Define the output variables and set to NaN

range_index       = ones(1,1) .* NaN;
Doppler_index     = ones(1,1) .* NaN;
peak_value        = ones(1,1) .* NaN;
x_value           = ones(1,1) .* NaN;
y_value           = ones(1,1) .* NaN;
z_value           = ones(1,1) .* NaN;
eof_flag          = 0;

%% get the object information:

read_value        = fread(fid,1,'uint16');
if(~isempty(read_value))
   numDetectObj      = read_value;
else
   eof_flag    = 1;
   return
end % end if(~isempty(read_value))

read_value        = fread(fid,1,'uint16');
if(~isempty(read_value))
   Q_factor       = 2.^read_value;
else
   eof_flag    = 1;
   return
end % end if(~isempty(read_Q_factor))

%% Define the output variables and set to NaN

range_index       = ones(numDetectObj,1) .* NaN;
Doppler_index     = ones(numDetectObj,1) .* NaN;
peak_value        = ones(numDetectObj,1) .* NaN;
x_value           = ones(numDetectObj,1) .* NaN;
y_value           = ones(numDetectObj,1) .* NaN;
z_value           = ones(numDetectObj,1) .* NaN;

%% Read each detected object

for r = 1:numDetectObj
 
   read_value        = fread(fid,1,'uint16');
   if(~isempty(read_value))   
      range_index(r)    = read_value;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))   
   
   read_value        = fread(fid,1,'int16');
   if(~isempty(read_value))   
      Doppler_index(r)  = read_value;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))   
   
   read_value        = fread(fid,1,'uint16');
   if(~isempty(read_value))   
      peak_value(r)     = read_value;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))   
   
   read_value        = fread(fid,1,'int16');
   if(~isempty(read_value))   
      x_value(r)        = read_value ./ Q_factor;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))   
   
   read_value        = fread(fid,1,'int16');
   if(~isempty(read_value))
      y_value(r)        = read_value ./ Q_factor;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))   
   
   read_value        = fread(fid,1,'int16');
   if(~isempty(read_value))   
      z_value(r)        = read_value ./ Q_factor;
   else
      eof_flag       = 1;
   end % end if(~isempty(read_value))   
   
end % end for r loop
