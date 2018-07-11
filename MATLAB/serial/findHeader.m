function [version, totalPacketLen, platform, frameNumber, timeCPUcycles, ...
        numObj, numTLV, subFrameNum, addToOffset] ...
        = findHeader(offset, D, numReadTLV, debug, count)
    
    %splits up the data packet header
    version        = typecast(uint8(D(offset      : offset +  3)), 'uint32');
    totalPacketLen = typecast(uint8(D(offset + 4  : offset +  7)), 'uint32');
    platform       = typecast(uint8(D(offset + 8  : offset + 11)), 'uint32');
    frameNumber    = typecast(uint8(D(offset + 12 : offset + 15)), 'uint32');
    timeCPUcycles  = typecast(uint8(D(offset + 16 : offset + 19)), 'uint32');
    numObj         = typecast(uint8(D(offset + 20 : offset + 23)), 'uint32');
    numTLV         = typecast(uint8(D(offset + 24 : offset + 27)), 'uint32');
    subFrameNum    = typecast(uint8(D(offset + 28 : offset + 31)), 'uint32');
    
    addToOffset = 32;
    
    if numTLV ~= numReadTLV && count == 1
        disp('Incorrect options chosen for TLV');
    end
    
    if debug
        disp('Header');
    end
end