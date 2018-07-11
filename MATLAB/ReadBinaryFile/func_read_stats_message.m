function [interFrameProcessingTime, transmitOutputTime, ...
   interFrameProcessingMargin, interChirpProcessingMargin, ...
   activeFrameCPULoad, interFrameCPULoad, eof_flag] = ...
   func_read_stats_message(fid)

% This function reads the Stats Message tag

% updated: 03-July-2018
% =======================================================================

%% Reset the eof_flag

eof_flag    = 0;

%% Each each value

% uint32_t 	interFrameProcessingTime
%  	Interframe processing time in usec. More...
read_value  = fread(fid,1,'uint32');
if(~isempty(read_value))
   interFrameProcessingTime = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
%disp(['interFrameProcessingTime: ',num2str(interFrameProcessingTime)])
%  
% uint32_t 	transmitOutputTime
%  	Transmission time of output detection informaion in usec. More...
read_value  = fread(fid,1,'uint32');
if(~isempty(read_value))
   transmitOutputTime = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
%disp(['transmitOutputTime: ',num2str(transmitOutputTime)])
%  
% uint32_t 	interFrameProcessingMargin
%  	Interframe processing margin in usec. More...
read_value  = fread(fid,1,'uint32');
if(~isempty(read_value))
   interFrameProcessingMargin = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
%disp(['interFrameProcessingMargin: ',num2str(interFrameProcessingMargin)])
%  
% uint32_t 	interChirpProcessingMargin
%  	Interchirp processing margin in usec. More...
read_value  = fread(fid,1,'uint32');
if(~isempty(read_value))
   interChirpProcessingMargin = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
%disp(['interChirpProcessingMargin: ',num2str(interChirpProcessingMargin)])
%  
% uint32_t 	activeFrameCPULoad
%  	CPU Load (%) during active frame duration. More...
read_value  = fread(fid,1,'uint32');
if(~isempty(read_value))
   activeFrameCPULoad = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
%disp(['activeFrameCPULoad: ',num2str(activeFrameCPULoad)])
%  
% uint32_t 	interFrameCPULoad
%  	CPU Load (%) during inter frame duration. More...
read_value  = fread(fid,1,'uint32');
if(~isempty(read_value))
   interFrameCPULoad = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
%disp(['interFrameCPULoad: ',num2str(interFrameCPULoad)])
