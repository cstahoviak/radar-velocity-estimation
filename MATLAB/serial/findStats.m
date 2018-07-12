function [tag, len, PAYLOAD, addToOffset] = findStats(offset, D)
    
    tag     = typecast(uint8(D(offset : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 31);
    
    addToOffset = len;
   
end