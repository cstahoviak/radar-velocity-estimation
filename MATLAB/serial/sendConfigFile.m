function [ ] = sendConfigFile( serial,filename,debug)
    %UNTITLED2 Read and send configuration file to device over serial port
    %   Detailed explanation goes here
    
    text  = fileread(filename);
    lines = strsplit(text,'\n');
    
    for i=1:length(lines)
        if strncmp(string(lines{i}), '%', 1) && debug
            fprintf('Ignoring string: %s\n', string(lines{i}));
        else
            if debug
                fprintf('Sent: %s\n',string(lines{i}));
            end
            
            fprintf(serial, lines{i});
            out = fgets(serial);
            
            if debug
                fprintf('Recieved: %s\n',out);
            end
            
            out = fgets(serial);
            
            if debug
                fprintf('Recieved: %s\n',out);
            end
        end
    end
    if debug
        disp('Done');
    end
end

