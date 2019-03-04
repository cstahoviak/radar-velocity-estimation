% program: main_read_demo_file_all_packets.m
% updated: 03-July-2018

% This routine reads all packets in a mmWave Demo Visualizer binary file.


%% Part 1. Initial Setup
% =======================

%% Define the filename

directory_filename   = '..\raw_data\';
%filename_root        = 'xwr16xx_processed_stream_2018_05_30T23_00_03_066.dat';
%filename_root        = 'xwr16xx_processed_stream_2018_05_16T23_44_54_538.dat';
filename_root        = 'xwr16xx_processed_stream_2018_07_03T12_43_40_598.dat';
filename             = [directory_filename,filename_root];

%% Verify that the filename is valid

good_filename  = exist(filename,'file');
if(good_filename == 2)
    disp(['reading file: ',filename,'...']);
else
    disp(['not a valid filename: ',filename,'...program paused...']);
    pause
end % end if(good_filename == 2)

%% Define the radar model (used to define Q factor)

radar_model    = 16;    % for AWR1642
%radar_model    = 14;   % for AWR1443

%% Define the number of virtual antennas (Static Azimuth Heat Map)

% The azimuth heat maps provide complex values (imag, real) for each
% (range,Rx(Tx#,Rx#))

% Need to define the number of virtual antennas because this value is not
% in the file.

num_virtual_antennas    = 8;

%% Define size of Doppler velocity (Range-Doppler Heat Map)

% The range-Doppler heat maps provide power in (R,Doppler velocity) space.

% Need to define the size of the Doppler velocity because this value is not
% in the file.

num_Doppler_bins  =  256;

%% Open the file

fid   = fopen(filename,'r');

%% Part 2. Determine how many valid packets are in this file
% ==========================================================

%% Set the counter to zero

packet_count   = 0;

%% Read each packet to determine how many packets are in this file.

% Read each packet until reaching the eof.
% eof_flag == 0 => not at end-of-file
% eof_flag == 1 => reached end-of-file while reading packet

keep_reading_packets    = 1;
max_numDetectedObj      = 0;
max_length_range_fft    = 0;

while(keep_reading_packets)
   
   %% Read a packet of data
   
   [version, totalPacketLen, platform, frameNumber, timeCpuCycles, ...
      numDetectedObj, numTLVs, subFrameNumber, ...
      detect_numDetectObj, detect_Q_factor, ...
      detect_range_index, detect_Doppler_index, detect_peak_value, ...
      detect_x_value, detect_y_value, detect_z_value, ...
      range_fft, noise_fft, static_azimuth_heat_map, range_Doppler_heat_map, ...
      interFrameProcessingTime, transmitOutputTime, ...
      interFrameProcessingMargin, interChirpProcessingMargin, ...
      activeFrameCPULoad, interFrameCPULoad, eof_flag] = func_read_demo_file_packet(fid, ...
      radar_model, num_virtual_antennas, num_Doppler_bins);
   
   if(eof_flag < 0.5)
      
      packet_count   = packet_count + 1;
      %disp(['packet_count: ',num2str(packet_count)]);

      % keep track of the maximum number of objects detected
      if(numDetectedObj > max_numDetectedObj)
         max_numDetectedObj   = numDetectedObj;
      end % end if(numDetectedObj > max_numDetectedObj)

      % keep track of the maximum length of range_fft
      if(length(range_fft) > max_length_range_fft)
         max_length_range_fft = length(range_fft);
      end % end if(numDetectedObj > max_numDetectedObj)
      
   else
      keep_reading_packets    = 0;
   
   end % end if(eof_flag < 0.5)
   
end % end while(keep_reading_packets)

%% Disply message stating the number of packets in file

% display a message:
disp(['There are ',num2str(packet_count),' packets in this file...']);

%% Part 3. Read all packets of data in this file
% ==============================================

%% Read all of the data in this file

keep_frame_number       = ones(packet_count,1) .* NaN;
keep_time_CPU_cycles    = ones(packet_count,1) .* NaN;
keep_range_index        = ones(packet_count, max_numDetectedObj) .* NaN;
keep_Doppler_index      = ones(packet_count, max_numDetectedObj) .* NaN;
keep_peak_value         = ones(packet_count, max_numDetectedObj) .* NaN;
keep_x_value            = ones(packet_count, max_numDetectedObj) .* NaN;
keep_y_value            = ones(packet_count, max_numDetectedObj) .* NaN;
keep_z_value            = ones(packet_count, max_numDetectedObj) .* NaN;

keep_range_fft          = ones(packet_count, max_length_range_fft) .* NaN;
keep_noise_fft          = ones(packet_count, max_length_range_fft) .* NaN;

keep_static_azimuth_heat_map = ones(packet_count, max_length_range_fft, num_virtual_antennas) .* NaN;

keep_range_Doppler      = ones(packet_count, max_length_range_fft, num_Doppler_bins) .* NaN;

%% Rewind the pointer to be at the beginning of the file

frewind(fid);

%% Read each packet

for r = 1:packet_count

   %% Read a packet of data
   
   [version, totalPacketLen, platform, frameNumber, timeCpuCycles, ...
      numDetectedObj, numTLVs, subFrameNumber, ...
      detect_numDetectObj, detect_Q_factor, ...
      detect_range_index, detect_Doppler_index, detect_peak_value, ...
      detect_x_value, detect_y_value, detect_z_value, ...
      range_fft, noise_fft, static_azimuth_heat_map, range_Doppler_heat_map, ...
      interFrameProcessingTime, transmitOutputTime, ...
      interFrameProcessingMargin, interChirpProcessingMargin, ...
      activeFrameCPULoad, interFrameCPULoad, eof_flag] = func_read_demo_file_packet(fid, ...
      radar_model, num_virtual_antennas, num_Doppler_bins);

   % save the values
   keep_frame_number(r)                         = frameNumber;
   keep_time_CPU_cycles(r)                      = timeCpuCycles;
   
   keep_range_index(r,1:detect_numDetectObj)    = detect_range_index;
   keep_Doppler_index(r,1:detect_numDetectObj)  = detect_Doppler_index;
   keep_peak_value(r,1:detect_numDetectObj)     = detect_peak_value;
   keep_x_value(r,1:detect_numDetectObj)        = detect_x_value;
   keep_y_value(r,1:detect_numDetectObj)        = detect_y_value;
   keep_z_value(r,1:detect_numDetectObj)        = detect_z_value;
   
   keep_range_fft(r,:)                          = range_fft;
   keep_noise_fft(r,:)                          = noise_fft;

   keep_static_azimuth_heat_map(r,:,:)          = static_azimuth_heat_map;

   keep_range_Doppler(r,:,:)                    = range_Doppler_heat_map;
   
end % end for r loop

%% Close the file

status   = fclose(fid);

%% Plot the range FFTs

figure
colormap('jet')
plot(keep_range_fft(1,:),'k')
hold on
plot(keep_noise_fft(1,:),'g')
legend('range FFT','noise FFT','location','NorthEast')

xlabel('range index')
ylabel('power')

%for r = 2:packet_count
%   plot(keep_range_fft(1,:),'r')   
%end % end for r loop
title('Range FFT')

%% plot azimuth heat map

figure
colormap('jet')

azimuth_heat_map    = squeeze(keep_static_azimuth_heat_map(1,:,:));

% extend the diminsion to plot the last value
[m,n] = size(azimuth_heat_map);
azimuth_heat_map(:,9) = azimuth_heat_map(:,8);

mag_heat_map  = 10.*log10(abs(azimuth_heat_map));
ang_heat_map  = angle(azimuth_heat_map);
f = ang_heat_map < 0;
if(sum(sum(f)) > 0)
    ang_heat_map(f)     = ang_heat_map(f) + 2*pi;
end % end if(sum(sum(f)) > 0)

subplot(2,1,1)
pcolor((1:n+1)-0.5,1:m,mag_heat_map);
shading flat
colorbar
axis([0 9 0 128])
xlabel('Rx#')
ylabel('range index')
title('a. Static Azimuth Heat Map, Magnitude, dB')

subplot(2,1,2)
pcolor((1:n+1)-0.5,1:m,ang_heat_map);
shading flat
colorbar
axis([0 9 0 128])
xlabel('Rx#')
ylabel('range index')
title('b. Static Azimuth Heat Map, Angle, Radian [0-2\pi]')
