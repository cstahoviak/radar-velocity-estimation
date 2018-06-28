%% Setting Up and Opening Serial Ports
debug = true;

command_port = serial('/dev/ttyACM0');
data_port = serial('/dev/ttyACM1');

command_rate = 115200;
data_rate = 921600;

command_port.BaudRate = command_rate;
data_port.BaudRate = data_rate;

%This is currently an arbitrary number that is definately more than one
%packet
data_port.InputBufferSize = 2048;

finish = onCleanup(@() close(data_port,command_port));

fopen(command_port);
fopen(data_port);

sendConfigFile(command_port,'bestRangeRes.cfg');

%% Reading in Data and Finding Magic Word

D = fread(data_port); %load('data.mat'); %this can be switched to load D from file

%Find Magic Word
offset = findMagicWord(D,1);

if offset == -1
    return;
end
if debug
    disp('Magic Word Found');
end

%% Sets different variables for different preferences in config file

rangeProfile = true;
noiseProfile = false;
rangeAzimuthHeatMap = true;
rangeDopplerHeatMap = false;
stats = true;

qFormat = 9;

rangeRes = 0.044;
maxRange = 9.02;

radialVelRes = 0.13;
maxRadialVel = 1;

numRXantenna = 4;
numTXantenna = 2;

numDopplerBins = maxRadialVel / radialVelRes;
numRangeBins = maxRange / rangeRes;
numVirtualAntenna = numRXantenna * numTXantenna;

addToOffset = 0;

%% Storing Various Header Components

VERSION = D(offset : offset + 3);
TOTAL_PACKET_LENGTH = D(offset + 4 : offset + 7);
PLATFORM = D(offset + 8 : offset + 11);
FRAME_NUMBER = D(offset + 12 : offset +15);
TIME_CPU_CYCLES = D(offset + 16 : offset + 19);
NUMBER_DETECTED_OBJECTS = D(offset + 20 : offset + 23);
NUMBER_TLV = D(offset + 24 : offset + 27);
addToOffset = addToOffset + 28;

%Convert from Bytes to standard numbers
header.version        = typecast(uint8(VERSION), 'uint32');
header.totalPacketLen = typecast(uint8(TOTAL_PACKET_LENGTH),'uint32');
header.platform       = typecast(uint8(PLATFORM),'uint32');
header.frameNumber    = typecast(uint8(FRAME_NUMBER),'uint32');
header.timeCPUcycles  = typecast(uint8(TIME_CPU_CYCLES),'uint32');
header.numObj         = typecast(uint8(NUMBER_DETECTED_OBJECTS), 'uint32');
header.numTLV         = typecast(uint8(NUMBER_TLV),'uint32');
if debug
    disp('Header');
end

%% Detected Objects

offset = offset + addToOffset;
DET_OBJ_TAG = D(offset : offset + 3);
DET_OBJ_LEN = D(offset +4 : offset + 7);
DET_OBJ_DESCRIPTOR = D(offset + 8 : offset + 11);
%assures that this will not have an ending index smaller than start index
if header.numObj > 0
    DET_OBJ_PAYLOAD = D(offset + 12 : offset + 11 + header.numObj * 12);
    point = struct;
    temp = zeros(3);
    for i = 1 : header.numObj
        difference = (i-1) * 12;
        point(i).rangeIdx   = typecast(uint8(DET_OBJ_PAYLOAD(difference + 1  : difference + 2 )), 'uint16');
        point(i).dopplerIdx = typecast(uint8(DET_OBJ_PAYLOAD(difference + 3  : difference + 4 )), 'uint16');
        point(i).peakVal    = typecast(uint8(DET_OBJ_PAYLOAD(difference + 5  : difference + 6 )), 'uint16');
        temp(1)      = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 7  : difference + 8 )), 'uint16'));
        temp(2)      = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 9  : difference + 10)), 'uint16'));
        temp(3)      = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 11 : difference + 12)), 'uint16'));
        
        for j = 1 : 3
            if temp(j) > 32767
                temp(j) = temp(j) - 65535;
            end
            temp(j) = temp(j) / (2 ^ qFormat);
        end
        
        point(i).x = temp(1);
        point(i).y = temp(2);
        point(i).z = temp(3);
    end
end
addToOffset = addToOffset + 12 + header.numObj * 12;
if debug
    disp('Detected Objects');
end

%% Range Profile

if rangeProfile
    offset = offset + addToOffset;
    RANGE_TAG = D(offset : offset + 3);
    RANGE_LEN = D(offset + 4 : offset + 7);
    RANGE_PAYLOAD = D(offset + 8 : offset + 7 + numRangeBins * 2);
    addToOffset = addToOffset + 8 + numRangeBins * 2;
    if debug
        disp('Range Profile');
    end
end


%% Noise Profile

if noiseProfile
    offset = offset + addToOffset;
    NOISE_TAG = D(offset : offset + 3);
    NOISE_LEN = D(offset + 4 : offset + 7);
    NOISE_PAYLOAD = D(offset + 8 : offset + 7 + numRangeBins * 2);
    addToOffset = addToOffset + 8 + numRangeBins * 2;
    if debug
        disp('Noise Profile');
    end
end


%% Azimuth Heat Map

if rangeAzimuthHeatMap
    offset = offset + addToOffset
    AZIMUTH_TAG = D(offset : offset + 3);
    AZIMUTH_LEN = D(offset + 4 : offset + 7);
    AZIMUTH_PAYLOAD = D(offset + 8 : offset + 7 + (numRangeBins * numVirtualAntenna * 4));
    addToOffset = addToOffset + 8 + (numRangeBins * numVirtualAntenna * 4);
    
    if debug
        disp('Azimuth Heat Map');
    end
end


%% Range Doppler Heat map

if rangeDopplerHeatMap
    offset = offset + addToOffset;
    RANGE_DOPPLER_TAG = D(offset : offset + 3);
    RANGE_DOPPLER_LEN = D(offset + 4 : offset + 7);
    RANGE_DOPPLER_PAYLOAD = D(offset + 8 : offset + 7 + (numRangeBins * numDopplerBins * 2));
    addToOffset = addToOffset + 8 + (numRangeBins * numDopplerBins * 2);
    
    if debug
        disp('Range Doppler Heat Map');
    end
end

%% Statistics Profile

if stats
    offset = offset + addToOffset;
    STAT_TAG = D(offset : offset + 3);
    STAT_LEN = D(offset + 4 : offset + 7);
    STAT_PAYLOAD = D(offset + 8 : offset + 31);
    addToOffset = addToOffset + 32;
    
    if debug
        disp('Statistics Profile');
    end
end

%% Where next search for magic word needs to start

offset = offset + addToOffset;

%% Closing and Clearing Serial Ports

function close(port1,port2)
    fclose(port1);
    fclose(port2);
    delete(port1);
    delete(port2);
    clear port1;
    clear port2;
end
