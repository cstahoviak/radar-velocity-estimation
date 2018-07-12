%% General Setup

clear;

%set up radar data figure
fig1 = figure(1);
ax1 = axes('Parent',fig1);

%Turns on and off print outs
debug = false;
%Condition main while loop takes
running = true;

%count to keep track of how many loops have been done, used to exit program after endCount loops
count = 1;
endCount = 1000;

%check to make sure that selected number of TLVs is correct
numReadTLV = 0;

%initializes data, D and the input buffer size
data = struct;
D = [];

%% Serial Port Setup

bufferSize = 4096; %this is long enough to never fill up

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

%open the ports
fopen(command_port);
fopen(data_port);

%shutdown procedure that closes the ports even if program fails or is stopped early
finish = onCleanup(@() close(data_port,command_port));

%sets the path to the config files, just change the last input argument to change cfg files
filePattern = fullfile('./cfg/', 'bestRangeRes.cfg');

%sends the specified config file to the device. The files are exported from the demo visualizer
[rangeRes, maxRange, radialVelRes, maxRadialVel] = sendConfigFile(command_port,filePattern);

%% Sets different variables from different preferences in config file

%these are the check marks at the bottom left of demo vizualizer software
scatterPlot = true;
rangeProfile = true;
noiseProfile = false;
rangeAzimuthHeatMap = false;
rangeDopplerHeatMap = false;
stats = false;

numRXantenna = 4;
numTXantenna = 2;

%Checks TLVs - used later as a check to make sure settings correct
numReadTLV = scatterPlot + rangeProfile + noiseProfile + rangeAzimuthHeatMap ...
    + rangeDopplerHeatMap + stats;

%Calcualations to find the number of bins for doppler and range and the number of virtual antenna
numDopplerBins = maxRadialVel / radialVelRes;
numRangeBins = maxRange / rangeRes;
numVirtualAntenna = numRXantenna * numTXantenna;

%% Reading in Data and Finding Magic Word

%loops until program is killed
while running
    
    firstTime = true;
    %a full packet will definately be held in 1000 characters or clear out the buffer if backed up
    while length(D) < 1000 || firstTime
        firstTime = false;
        pause(0.02);   %this pause prevents this loop from exicuting too fast
        
        if data_port.BytesAvailable > 200   %if there are more than 100 bytes available read in the data and add to end of D
            D = [D; fread(data_port,data_port.BytesAvailable)];
        end
    end
    
    %Find Magic Word
    offset = findMagicWord(D,1);
    
    if debug
        disp('Magic Word Found');
    end
    
    %% Storing Various Header Components
    
    [data(count).header.version, data(count).header.totalPacketLen, ...
        data(count).header.platform, data(count).header.frameNumber, ...
        data(count).header.timeCPUcycles, data(count).header.numObj, ...
        data(count).header.numTLV, data(count).header.subFrameNum, addToOffset] ...
        = findHeader(offset, D, numReadTLV, debug, count);
    
    %% Detected Objects
    
    if scatterPlot
        
        offset = offset + addToOffset;
        
        [data(count).detObj.tag, data(count).detObj.len, data(count).detObj.numObj, data(count).detObj.qFormat, ...
            data(count).detObj.point, addToOffset] ...
            = findDetectedObj(offset, D, debug);
       
    end
    %% Range Profile
    
    if rangeProfile
        
        offset = offset + addToOffset;
        
        [data(count).rangeProfile.tag, data(count).rangeProfile.len, ...
            data(count).rangeProfile.PAYLOAD, addToOffset] ...
            = findRangeProfile(offset, D, debug, numRangeBins);
    end
    
    
    %% Noise Profile
    
    if noiseProfile
        
        offset = offset + addToOffset;
        
        [data(count).noiseProfile.tag, data(count).noiseProfile.len, ...
            data(count).noiseProfile.PAYLOAD, addToOffset] ...
            = findNoiseProfile(offset, D, debug);
    end
    
    
    %% Azimuth Heat Map
    
    if rangeAzimuthHeatMap
        
        offset = offset + addToOffset;
        
        [data(count).azimuthHeat.tag, data(count).azimuthHeat.len, ...
            data(count).azimuthHeat.PAYLOAD, addToOffset] ...
            = findAzimuthHeatMap(offset, D, debug, numRangeBins, numVirtualAntenna);
    end
    
    
    %% Range Doppler Heat map
    
    if rangeDopplerHeatMap
        
        offset = offset + addToOffset;
        
        [data(count).dopplerHeat.tag, data(count).dopplerHeat.len, ...
            data(count).dopplerHeat.PAYLOAD, addToOffset] ...
            = findDopplerHeatMap(offset, D, debug, numRangeBins, numDopplerBins);
    end
    
    %% Statistics Profile
    
    if stats
        
        offset = offset + addToOffset;
        
        [data(count).stats.tag, data(count).stats.len, ...
            data(count).stats.PAYLOAD, addToOffset] ...
            = findStats(offset, D, debug);
    end
    
    %% Plotting the points
    
    %plots the x,y and intencity - does it every other time in order to not cause a build up of data
    x = [data(count).detObj.point(:).x];
    y = [data(count).detObj.point(:).y];
    for i=1:length(x)
        intensity = double(data(count).detObj.point(i).peakVal)/8000;
        if intensity > 1
            intensity = 1;
        end
        if x(i) < 3 && x(i) > -3 && y(i) < 10 && intensity > 0.1
            scatter(ax1,x(i),y(i),'MarkerFaceColor','b', ...
            'MarkerEdgeColor','b','MarkerFaceAlpha',intensity, ...
            'MarkerEdgeAlpha',0)
            xlim(ax1,[-3,3]); ylim(ax1,[0,5]);
            hold on;
        end
    end
    hold off;
    
    %% Clear out data and check count
    
    %clears out the used data packet
    D(1:data(count).header.totalPacketLen) = [];
    
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
