function [version, totalPacketLen, platform, frameNumber, timeCpuCycles, ...
   numDetectedObj, numTLVs, subFrameNumber, numDetectObj, Q_factor, ...
   range_index, Doppler_index, peak_value, x_value, y_value, z_value, ...
   range_fft, noise_fft, static_azimuth_heat_map, range_Doppler_heat_map, ...
   interFrameProcessingTime, transmitOutputTime, ...
   interFrameProcessingMargin, interChirpProcessingMargin, ...
   activeFrameCPULoad, interFrameCPULoad, eof_flag] = func_read_demo_file_packet(fid, ...
   radar_model, num_virtual_antennas, num_Doppler_bins)

% updated: 03-July-2018
% =======================================================================

%% Define output variables and set to NaN

% These variables will be read from the header
%version                       = NaN;
%totalPacketLen                = NaN;
%platform                      = NaN;
%frameNumber                   = NaN;
%timeCpuCycles                 = NaN;
%numDetectedObj                = NaN;
%numTLVs                       = NaN;
%subFrameNumber                = NaN;
numDetectObj                  = 0;
Q_factor                      = NaN;
range_index                   = NaN;
Doppler_index                 = NaN;
peak_value                    = NaN;
x_value                       = NaN;
y_value                       = NaN;
z_value                       = NaN;
range_fft                     = NaN;
noise_fft                     = NaN;
static_azimuth_heat_map       = NaN;
range_Doppler_heat_map        = NaN;
interFrameProcessingTime      = NaN;
transmitOutputTime            = NaN;
interFrameProcessingMargin    = NaN;
interChirpProcessingMargin    = NaN;
activeFrameCPULoad            = NaN;
interFrameCPULoad             = NaN;

%% Need to find the magic word. Then you are at the beginning of packet

search_for_magic_word_flag    = 1;

% Reset the eof flag to zero
eof_flag                      = 0;

while(search_for_magic_word_flag && ~eof_flag)
   
   %% read values until you read the magic word
   
   read_value   = fread(fid,1,'uint16');
   if(~isempty(read_value))
      if(read_value == hex2dec('102'))
         read_value   = fread(fid,1,'uint16');
         if(~isempty(read_value))
            if(read_value == hex2dec('304'))
               read_value   = fread(fid,1,'uint16');
               if(~isempty(read_value))
                  
                  if(read_value == hex2dec('506'))
                     read_value   = fread(fid,1,'uint16');
                     if(~isempty(read_value))
                  
                        if(read_value == hex2dec('708'))
                           % if you get here, then you just read the magic word
                           search_for_magic_word_flag = 0;
                        end % end if(read_temp == hex2dec('708'))
                     else
                        eof_flag    = 1;
                        return
                     end % end if(~isempty(read_value))
                     
                  end % end if(read_temp == hex2dec('506'))
               else
                  eof_flag    = 1;
                  return
               end % end if(~isempty(read_value))

            end % end if(read_temp == hex2dec('304'))
         else
            eof_flag    = 1;
            return
         end % end if(~isempty(read_value))
      end % end if(read_temp == hex2dec('102'))
   else
      eof_flag    = 1;
      return
   end % end if(~isempty(read_value))
   
end % end while(search_for_magic_word_flag && ~eof_flag)

%% If you get here, then you found a magic word

%% Read the header

% read the header
[version, totalPacketLen, platform, frameNumber, timeCpuCycles, ...
   numDetectedObj, numTLVs, subFrameNumber, eof_flag] = func_read_header(fid);

% Keep track of the packet length as you go
current_packet_length = 40; % length of header including magic word

% The number of TLVs is needed to know how many TLVs to read.

%% Each each TLV

for r = 1:numTLVs
   
   %% Read the TVL header
   
   [tag_type, tag_length, eof_flag] = func_read_TLV_header(fid);
   current_packet_length = current_packet_length + 8;
   
   %% Verify that tag_length is valid (else at eof)
   
   if(~isempty(tag_length))
      
      %% Depending on tag_type, read different TLV structures
      
      % {change this logic to "switch" / "case" later...}
      
      %% Detected Objects TVL
      
      if(tag_type == 1)
         [numDetectObj, Q_factor, range_index, Doppler_index, peak_value, ...
            x_value, y_value, z_value, eof_flag] = func_read_object_information(fid);
         
         %current_packet_length = current_packet_length + 4;
         %current_packet_length = current_packet_length + numDetectObj*12;
         current_packet_length = current_packet_length + tag_length;
         
      end % end if(tag_type == 1)
      
      %% Range Profile TVL
      
      if(tag_type == 2)
         [range_fft, eof_flag] = func_read_range_profile(fid, radar_model, tag_length);
         
         % tag_length = (number FFT points)*2
         current_packet_length = current_packet_length + tag_length;
         
      end % end if(tag_type == 1)
      
      %% Range Profile TVL
      
      if(tag_type == 3)
         [noise_fft, eof_flag] = func_read_noise_profile(fid, radar_model, tag_length);
         
         % tag_length = (number FFT points)*2
         current_packet_length = current_packet_length + tag_length;
         
      end % end if(tag_type == 1)
      
      %% Static Azimuth Heat Map TVL
      
      if(tag_type == 4)
         [static_azimuth_heat_map, eof_flag] = func_read_static_azimuth_heat_map(fid, num_virtual_antennas, tag_length);
         
         % tag_length = (number FFT points)*2
         current_packet_length = current_packet_length + tag_length;
         
      end % end if(tag_type == 1)
      
      %% Range_Doppler Heat Map TVL
      
      if(tag_type == 5)
         [range_Doppler_heat_map, eof_flag] = func_read_range_Doppler_heat_map(fid, num_Doppler_bins, tag_length);
         
         % tag_length = (number FFT points)*2
         current_packet_length = current_packet_length + tag_length;
         
      end % end if(tag_type == 1)
      
      
      %% Stats Message TVL
      
      if(tag_type == 6)
         [interFrameProcessingTime, transmitOutputTime, ...
            interFrameProcessingMargin, interChirpProcessingMargin, ...
            activeFrameCPULoad, interFrameCPULoad, eof_flag] = ...
            func_read_stats_message(fid);
         
         % tag_length = (6 values)*4 = 24
         current_packet_length = current_packet_length + tag_length;
         
      end % end if(tag_type == 1)
      
   end % end if(isempty(tag_length) > 0)
   
end % end for r loop

%% Get the number of padding Bytes

padding_words  = totalPacketLen - current_packet_length;

num_extra_reads   = round(padding_words/2);

%% read extra words

for r = 1:num_extra_reads
   read_value    = fread(fid,1,'uint16');
   if(~isempty(read_value))
      %dumby = read_value;
      %disp(['dumby, r: ',num2str(r),', dumby: ',num2str(dumby)]);
   else
      eof_flag    = 1;
   end % end if(~isempty(read_value))
end % end for r loop

