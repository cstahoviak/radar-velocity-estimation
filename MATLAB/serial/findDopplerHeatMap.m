function [tag, len, PAYLOAD, addToOffset] ...
        = findDopplerHeatMap(offset, D, numRangeBins, numDopplerBins)
    
    tag     = typecast(uint8(D(offset : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 7 +(numRangeBins * numDopplerBins * 2));
    
    addToOffset = len;
    
end