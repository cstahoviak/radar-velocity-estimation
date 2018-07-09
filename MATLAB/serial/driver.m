%% Setting Up and Opening Serial Ports

%Turns on and off print outs
debug = false;
%Condition main while loop takes
running = true;

%count to keep track of how many loops have been done, used to exit program after endCount loops
count = 1;
endCount = 100;

%initializes data, D and the input buffer size
data = struct;
D = [];
bufferSize = 2048; %this is long enough to never fill up

%set the command and data ports
command_port = serial('/dev/ttyACM0');
data_port = serial('/dev/ttyACM1');

%set the baud rates of the different ports
command_rate = 115200;
data_rate = 921600;

command_port.BaudRate = command_rate;
data_port.BaudRate = data_rate;

%This is currently an arbitrary number that is definately more than one packet
data_port.InputBufferSize = bufferSize;

%shutdown procedure that closes the ports even if program fails or is stopped early
finish = onCleanup(@() close(data_port,command_port));

%open the ports
fopen(command_port);
fopen(data_port);
%allow the data to be continually read even when the data processing is happening
readasync(data_port);

%sends the specified config file to the device. The files are exported from the demo visualizer
sendConfigFile(command_port,'bestRangeRes.cfg',debug);

%% Sets different variables for different preferences in config file

%these are the check marks at the bottom left of demo vizualizer software
rangeProfile = true;
noiseProfile = false;
rangeAzimuthHeatMap = false;
rangeDopplerHeatMap = false;
stats = false;

%q format used to convert from decimals to integers
qFormat = 9;

% from Demo Visualizer - will want to pull from cfg file evenatually
rangeRes = 0.044;
maxRange = 9.02;

% from Demo Visualizer - will want to pull from cfg file evenatually
radialVelRes = 0.13;
maxRadialVel = 1;

numRXantenna = 4;
numTXantenna = 2;

%Calcualations to find the number of bins for doppler and range and the number of virtual antenna
numDopplerBins = maxRadialVel / radialVelRes;
numRangeBins = maxRange / rangeRes;
numVirtualAntenna = numRXantenna * numTXantenna;

%% Reading in Data and Finding Magic Word

%loops until program is killed
while running
    
    %a full packet will definately be held in 2000 characters
    while length(D) < 2000
        pause(0.05);   %this pause prevents this loop from exicuting too fast
        if data_port.BytesAvailable > 100   %if there are more than 100 bytes available read in the data and add to end of D
            D = [D; fread(data_port,data_port.BytesAvailable)];
        end
    end
    
    % delete so input buffer does not fill up
    flushinput(data_port);
    
    offset = 1;
    
    if debug
        disp('Serial Port Read--------------------------------------');
    end
    
    %Find Magic Word
    offset = findMagicWord(D,offset);
    disp(offset);
    if debug
        disp('Magic Word Found');
    end
    
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
    data(count).header.version        = typecast(uint8(VERSION), 'uint32');
    data(count).header.totalPacketLen = typecast(uint8(TOTAL_PACKET_LENGTH),'uint32');
    data(count).header.platform       = typecast(uint8(PLATFORM),'uint32');
    data(count).header.frameNumber    = typecast(uint8(FRAME_NUMBER),'uint32');
    data(count).header.timeCPUcycles  = typecast(uint8(TIME_CPU_CYCLES),'uint32');
    data(count).header.numObj         = typecast(uint8(NUMBER_DETECTED_OBJECTS), 'uint32');
    data(count).header.numTLV         = typecast(uint8(NUMBER_TLV),'uint32');
    
    if debug
        disp('Header');
    end
    
    
    %% Detected Objects
   
    offset = offset + addToOffset;
    DET_OBJ_TAG = D(offset : offset + 3);
    DET_OBJ_LEN = D(offset +4 : offset + 7);
    DET_OBJ_DESCRIPTOR = D(offset + 8 : offset + 11);
    
    %assures that this will not have an ending index smaller than start index
    if data(count).header.numObj > 0
       
        % each detected object has 12 bytes worth of data (slide 3)
        DET_OBJ_PAYLOAD = D(offset + 12 : offset + 11 + data(count).header.numObj * 12);
        data(count).point = struct;
        temp = zeros(3);
        
        for i = 1 : data(count).header.numObj
            difference = (i-1) * 12;
            data(count).point(i).rangeIdx   = typecast(uint8(DET_OBJ_PAYLOAD(difference + 1  : difference + 2 )), 'uint16');
            data(count).point(i).dopplerIdx = typecast(uint8(DET_OBJ_PAYLOAD(difference + 3  : difference + 4 )), 'uint16');
            data(count).point(i).peakVal    = typecast(uint8(DET_OBJ_PAYLOAD(difference + 5  : difference + 6 )), 'uint16');
            temp(1)      = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 7  : difference + 8 )), 'uint16'));
            temp(2)      = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 9  : difference + 10)), 'uint16'));
            temp(3)      = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 11 : difference + 12)), 'uint16'));
            
            % math reproduced from ti_mmwave_rospkg/src/DataHandlerClass.cpp line 435
            for j = 1 : 3
                if temp(j) > 32767
                    temp(j) = temp(j) - 65535;
                end
                temp(j) = temp(j) / (2 ^ qFormat);
            end
            
            data(count).point(i).x = temp(1);
            data(count).point(i).y = temp(2);
            data(count).point(i).z = temp(3);
        end
    end
    
    %this accounts for the various lengths that the determined objects payload could be
    addToOffset = addToOffset + 12 + data(count).header.numObj * 12;
    
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
    
    %clears out the used data packet
    D(1:offset + addToOffset) = [];
    
    %increments count to keep track of number of loops done
    count = count + 1;
    
    if debug
        fprintf("Count: %d \n",count);
    end
    
    %ends program after endCount packets are collected
    if count > endCount
        running = false;
    end
end

%% Closing and Clearing Serial Ports

function close(port1,port2)
    fclose(port1);
    fclose(port2);
    delete(port1);
    delete(port2);
    clear port1;
    clear port2;
end
