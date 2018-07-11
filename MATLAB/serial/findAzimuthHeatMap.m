function [tag, len, PAYLOAD, addToOffset] ...
        = findAzimuthHeatMap(offset, D, debug, numRangeBins, numVirtualAntenna)
    
    tag     = typecast(uint8(D(offset : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 7 +(numRangeBins * numVirtualAntenna * 4));
    
    addToOffset = 8 + (numRangeBins * numVirtualAntenna * 4);
    
    if debug
        disp('Azimuth Heat Map');
    end
    
end