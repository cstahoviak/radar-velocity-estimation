function [tag, len, PAYLOAD, addToOffset] = findNoiseProfile(offset, D, debug)
    
    tag     = typecast(uint8(D(offset : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 7 + numRangeBins * 2);
    
    addToOffset =  8 + numRangeBins * 2;
    
    if debug
        disp('Noise Profile');
    end
    
end