function [tag, len, PAYLOAD, addToOffset] = findRangeProfile(offset, D, numRangeBins)
    
    tag     = typecast(uint8(D(offset    : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 7 + numRangeBins * 2);
    
    addToOffset = len;
    
end