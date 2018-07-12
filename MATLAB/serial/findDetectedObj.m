function [tag, len, numObj, qFormat, point, addToOffset] ...
        = findDetectedObj(offset, D)
    
    tag     = typecast(uint8(D(offset     : offset +  3)), 'uint32');
    len     = typecast(uint8(D(offset +  4: offset +  7)), 'uint32');
    numObj  = typecast(uint8(D(offset +  8: offset +  9)), 'uint16');
    qFormat = typecast(uint8(D(offset + 10: offset + 11)), 'uint16');
    
    %assures that this will not have an ending index smaller than start index
    if numObj > 0
        
        % each detected object has 12 bytes worth of data (slide 3)
        DET_OBJ_PAYLOAD = D(offset + 12 : offset + 11 + numObj * 12);
        temp = zeros(3);
        point = struct;
        
        for i = 1 : numObj
            
            difference = (i-1) * 12;
            
            point(i).rangeIdx   = typecast(uint8(DET_OBJ_PAYLOAD(difference + 1  : difference + 2 )), 'uint16');
            point(i).dopplerIdx = typecast(uint8(DET_OBJ_PAYLOAD(difference + 3  : difference + 4 )), 'uint16');
            point(i).peakVal    = typecast(uint8(DET_OBJ_PAYLOAD(difference + 5  : difference + 6 )), 'uint16');
            
            temp(1) = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 7  : difference + 8 )), 'uint16'));
            temp(2) = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 9  : difference + 10)), 'uint16'));
            temp(3) = double(typecast(uint8(DET_OBJ_PAYLOAD(difference + 11 : difference + 12)), 'uint16'));
            
            % math reproduced from ti_mmwave_rospkg/src/DataHandlerClass.cpp line 435
            for j = 1 : 3
                if temp(j) > 32767
                    temp(j) = temp(j) - 65535;
                end
                temp(j) = temp(j) / (2 ^ double(qFormat));
            end
            
            point(i).x = temp(1);
            point(i).y = temp(2);
            point(i).z = temp(3);
        end
    end
    
    %this accounts for the various lengths that the determined objects payload could be
    addToOffset = len;
    
end