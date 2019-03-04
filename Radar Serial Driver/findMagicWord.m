% Function to find start of data packet in serial buffer

function startPacket = findMagicWord(D,start)
    
    magicWord = [2,1,4,3,6,5,8,7];
    last8Bytes = D(start:start+7);
    index = start+8;
    
    while(~isMagicWord(last8Bytes,magicWord))
        for i = 1 : 7
            last8Bytes(i) = last8Bytes(i+1);
        end
        last8Bytes(8) = D(index);
        index = index + 1;
        if index > length(D)
            %disp('No Magic Word Found In Buffer');
            startPacket = -1;
            return;
        end
    end
    
    startPacket = index;
end

function r = isMagicWord(last8Bytes, magicWord)
    r = true;
    for i = 1:8
      if last8Bytes(i) ~= magicWord(i)
          r = false;
          return
      end
   end
end

            
       