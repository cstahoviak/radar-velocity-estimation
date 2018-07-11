function [version, totalPacketLen, platform, frameNumber, ...
   timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber, eof_flag] = ...
   func_read_header(fid)

% This routine reads the header *after* finding the magic word.

% updated: 03-July-2018
% =======================================================================

%% Reset the eof_flag

eof_flag    = 0;

%%

% The magic word has already been read. It is four 2-Byte words. (8 Bytes)
% and this length is included in the totalPacketLen
% *** Comment out this read, it is only here for completeness ***

% uint16_t 	magicWord [4]
%  	Output buffer magic word (sync word). It is initialized to {0x0102,0x0304,0x0506,0x0708}. More...
% comment out
%magicWord = fread(fid,4,'uint16');
%magicWord_hex = dec2hex(magicWord);


% uint32_t 	version
read_value   = fread(fid,1,'uint32');
if(~isempty(read_value))
   version = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
  
% uint32_t 	totalPacketLen
%  	Total packet length including header in Bytes. More...
read_value   = fread(fid,1,'uint32');
if(~isempty(read_value))
   totalPacketLen = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
  
% uint32_t 	platform
%  	platform type More...
read_value = fread(fid,1,'uint32');
if(~isempty(read_value))
   platform = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
  
% uint32_t 	frameNumber
%  	Frame number. More...
read_value = fread(fid,1,'uint32');
if(~isempty(read_value))
   frameNumber = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
  
% uint32_t 	timeCpuCycles
%  	Time in CPU cycles when the message was created. For XWR16xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles. More...
read_value = fread(fid,1,'uint32');
if(~isempty(read_value))
   timeCpuCycles = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
  
% uint32_t 	numDetectedObj
%  	Number of detected objects. More...
read_value = fread(fid,1,'uint32');
if(~isempty(read_value))
   numDetectedObj = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))

% uint32_t 	numTLVs
%  	Number of TLVs. More...
read_value = fread(fid,1,'uint32');
if(~isempty(read_value))
   numTLVs = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
  
% uint32_t 	subFrameNumber
%  	For Advanced Frame config, this is the sub-frame number in the range 0 to (number of subframes - 1). For frame config (not advanced), this is always set to 0. More...
read_value = fread(fid,1,'uint32');
if(~isempty(read_value))
   subFrameNumber = read_value;
else
   eof_flag    = 1;
end % end if(~isempty(read_value))
