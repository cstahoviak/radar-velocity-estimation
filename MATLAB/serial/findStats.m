function [tag, len, interFrameProcessingTime, transmitOutputTime, ...
        interFrameProcessingMargin, interChirpProcessingMargin, ...
        activeFrameCPULoad, interFrameCPULoad, addToOffset]...
        = findStats(offset, D)
    
    tag                        = typecast(uint8(D(offset     : offset +  3)), 'uint32');
    len                        = typecast(uint8(D(offset +  4: offset +  7)), 'uint32');
    interFrameProcessingTime   = typecast(uint8(D(offset +  8: offset + 11)), 'uint32');
    transmitOutputTime         = typecast(uint8(D(offset + 12: offset + 15)), 'uint32');
    interFrameProcessingMargin = typecast(uint8(D(offset + 16: offset + 19)), 'uint32');
    interChirpProcessingMargin = typecast(uint8(D(offset + 20: offset + 23)), 'uint32');
    activeFrameCPULoad         = typecast(uint8(D(offset + 24: offset + 27)), 'uint32');
    interFrameCPULoad          = typecast(uint8(D(offset + 28: offset + 31)), 'uint32');
    
    addToOffset = len;
   
end