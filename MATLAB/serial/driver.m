%% Header

% Make sure the have the IWR1642 connected to the serial ports before
% opening matlab.

% This will allow matlab to recognize and open them

%% General Setup

clear;

%set up radar data figure
fig1 = figure(1);
ax1 = axes('Parent',fig1);

%Condition main while loop takes
running = true;

%count to keep track of how many loops have been done, used to exit program after endCount loops
count = 1;
endCount = 100;

%initializes data and D
data = struct;
D = [];

%% Serial Port Setup

%set the command and data ports
command_port = serial('/dev/ttyACM0');
data_port = serial('/dev/ttyACM1');

%set the baud rates of the different ports
command_rate = 115200;
data_rate = 921600;

command_port.BaudRate = command_rate;
data_port.BaudRate = data_rate;

%Set the buffer size
bufferSize = 4096; %this is long enough to never fill up

data_port.InputBufferSize = bufferSize;

%open the ports
fopen(command_port);
fopen(data_port);

%shutdown procedure that closes the ports even if program fails or is stopped early
finish = onCleanup(@() close(data_port,command_port));

%sets the path to the config files, just change the last input argument to change cfg files
% SET THE CONFIG FILE CORRECTLY BEFORE EACH USE
% basic 1642 config is 1642_2d_noGrouping.cfg
% basic 1443 config is 1443_best_range_res.cfg
filePattern = fullfile('./cfg/', 'bothHeatMap_1642_1_2.cfg');

%sends the specified config file to the device. The files are exported from the demo visualizer
[rangeRes, maxRange, radialVelRes, maxRadialVel] = sendConfigFile(command_port,filePattern);

%% Set Number of Antenna and calculate various bins
% SET NUMBER OF ANTENNA CORRECTLY BEFORE EACH USE
% 1443 has 3 tx and 1642 has 2 tx
numRXantenna = 4;
numTXantenna = 2;

% Calcualations to find the number of bins for doppler and range and the number of virtual antenna
numDopplerBins = maxRadialVel / radialVelRes;
numRangeBins = maxRange / rangeRes;
numVirtualAntenna = numRXantenna * numTXantenna;

%% Looping through Serial Buffer

% Loops until program is killed
while running

    firstTime = true;
    % A full packet will definately be held in 1280 characters or clear out the buffer if backed up
    % This number needs to be re-calculated depending on the config file (want it to be as close to 1 packet as possible)
    while length(D) < 8000 || firstTime
        if ~firstTime
            pause(0.02);   %this pause prevents this loop from exicuting too fast, but on the first time though it will just read incase it needs to catch up
        end
        firstTime = false;

        if data_port.BytesAvailable > 200   %if there are more than 100 bytes available read in the data and add to end of D
            D = [D; fread(data_port,data_port.BytesAvailable)];
        end
    end

    %Find Magic Word
    offset = findMagicWord(D,1);

    %% Divide packet into different parts

    %Every packet has a header
    [data(count).header.version, data(count).header.totalPacketLen, ...
    data(count).header.platform, data(count).header.frameNumber, ...
    data(count).header.timeCPUcycles, data(count).header.numObj, ...
    data(count).header.numTLV, data(count).header.subFrameNum, addToOffset] ...
    = findHeader(offset, D);

    for i = 1 : data(count).header.numTLV

        offset = offset + addToOffset;
        %The various different TLVs have different tags associated with them
        tag = typecast(uint8(D(offset : offset + 3)), 'uint32');

        switch tag


        case 1

            [data(count).detObj.tag, data(count).detObj.len, data(count).detObj.numObj,...
            data(count).detObj.qFormat, data(count).detObj.point, addToOffset] ...
            = findDetectedObj(offset, D, radialVelRes);

        case 2
            %NOTE: Future work is to understand the PAYLOADs
            [data(count).rangeProfile.tag, data(count).rangeProfile.len, ...
            data(count).rangeProfile.PAYLOAD, addToOffset] ...
            = findRangeProfile(offset, D, numRangeBins);

        case 3


            [data(count).noiseProfile.tag, data(count).noiseProfile.len, ...
            data(count).noiseProfile.PAYLOAD, addToOffset] ...
            = findNoiseProfile(offset, D);

        case 4


            [data(count).azimuthHeat.tag, data(count).azimuthHeat.len, ...
            data(count).azimuthHeat.map, addToOffset] ...
            = findAzimuthHeatMap(offset, D, numRangeBins, numVirtualAntenna);

        case 5

            [data(count).dopplerHeat.tag, data(count).dopplerHeat.len, ...
            data(count).dopplerHeat.map, addToOffset] ...
            = findDopplerHeatMap(offset, D, numRangeBins, numDopplerBins);
        case 6

            [data(count).stats.tag, data(count).stats.len, data(count).stats.interFrameProcessingTime, ...
            data(count).stats.transmitOutputTime, data(count).stats.interFrameProcessingMargin, ...
            data(count).stats.interChirpProcessingMargin, data(count).stats.activeFrameCPULoad, ...
            data(count).stats.interFrameCPULoad, addToOffset] ...
            = findStats(offset, D);
        end
    end

    %% Plotting the points

    if length(D) < 3000
        %plots the x,y and intencity - does it every other time in order to not cause a build up of data
        if data(count).header.numObj > 0
            x = [data(count).detObj.point(:).x];
            y = [data(count).detObj.point(:).y];
            if data(count).header.platform ~= uint32(661058)
                z = [data(count).detObj.point(:).z];
            end
            for i=1:length(x)
                intensity = double(data(count).detObj.point(i).peakVal)/8000;
                if intensity > 1
                    intensity = 1;
                end
                if data(count).header.platform == uint32(661058)
                    if intensity > 0.1
                        scatter(ax1,x(i),y(i),'MarkerFaceColor','b', ...
                        'MarkerEdgeColor','b','MarkerFaceAlpha',intensity, ...
                        'MarkerEdgeAlpha',0)
                        xlim(ax1,[-3,3]); ylim(ax1,[0,5]);
                        hold on;
                    end
                else
                    if intensity > 0.1
                        scatter3(ax1,x(i),y(i),z(i),'MarkerFaceColor','b', ...
                        'MarkerEdgeColor','b','MarkerFaceAlpha',intensity, ...
                        'MarkerEdgeAlpha',0)
                        xlim(ax1,[-3,3]); ylim(ax1,[0,5]); zlim(ax1,[-1,1]);
                        hold on;
                        % This makes the elevation obvious
                        view(90,0);
                    end
                end
            end
            hold off;
        end
    end

    %% Clear out data and check count

    %clears out the used data packet
    D(1:data(count).header.totalPacketLen) = [];

    %increments count to keep track of number of loops done
    count = count + 1;

    %ends program after endCount packets are collected
    if count > endCount
        running = false;
    end

end

% Delete this figure at end of program
delete(fig1);
disp('Deleting Figure');
% Tells radar to stop transmitting
fprintf(command_port,'sensorStop');
disp('Sent: sensorStop');

%% Closing and Clearing Serial Port
function close(port1, port2)
    fclose(port1);
    fclose(port2);
    delete(port1);
    delete(port2);
    clear port1;
    clear port2;
end