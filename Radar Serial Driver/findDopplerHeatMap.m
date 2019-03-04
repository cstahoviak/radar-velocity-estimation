function [tag, len, map, addToOffset] ...
        = findDopplerHeatMap(offset, D, numRangeBins, numDopplerBins)
    
    tag     = typecast(uint8(D(offset : offset + 3)), 'uint32');
    len     = typecast(uint8(D(offset + 4: offset + 7)), 'uint32');
    PAYLOAD = D(offset + 8 : offset + 7 +(numRangeBins * numDopplerBins * 2));
    for i = 1 : 2 : length(PAYLOAD) - 1
        temp = typecast(uint8(PAYLOAD(i : i + 1)), 'uint16');
    end
    
    fftSize = round(len ./(numDopplerBins*2));
    map = ones(fftSize, numDopplerBins) .* NaN;
    
    for r = 1 : fftSize
        for c = 1 : numDopplerBins
            map(r,c) = temp(1);
            temp = temp(2:end);
        end
    end
    
    addToOffset = len;
    
end