function [tag, len, map, addToOffset] ...
        = findAzimuthHeatMap(offset, D, numRangeBins, numVirtualAntenna)
    
    tag     = typecast(uint8(D(offset : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 7 +(numRangeBins * numVirtualAntenna * 4));
    
    for i = 1 : 2 : length(PAYLOAD) - 1
        temp = typecast(uint8(PAYLOAD(i : i + 1)), 'uint16');
    end
    
    fftSize = round(tag_length ./ (num_virtual_antennas * 4));
    map = ones(fftSize, numVirtualAntenna) .* NaN;
    sqrtNeg1 = sqrt(-1);
    
    for r = 1 : fftSize
        for c = 1 : numVirtualAntenna
            
            imagValue = temp(1);
            realValue = temp(2);
            temp = temp(3:end);
            
            map(r,c) = realValue + sqrtNeg1 * imagValue;
            
        end
    end
            
    
    addToOffset = len;
    
end