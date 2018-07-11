function [tag_type, tag_length, eof_flag] = func_read_TLV_header(fid)

% This function reads the TLV header

% updated: 03-July-2018
% =======================================================================

%% Define the output variables

tag_type    = NaN;
tag_length  = NaN;

%% Rest the eof_flag to zero

eof_flag    = 0;

%% Read the tag_type and tag_value
% type, length, value
read_value        = fread(fid,1,'uint32');
if(~isempty(read_value))
   tag_type    = read_value;
else
   eof_flag       = 1;
end % end if(~isempty(read_value))

read_value        = fread(fid,1,'uint32');
if(~isempty(read_value))
   tag_length     = read_value;
else
   eof_flag       = 1;
end % end if(~isempty(read_value))
