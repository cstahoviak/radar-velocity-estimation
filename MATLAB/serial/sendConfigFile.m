function [ ] = sendConfigFile( serial,filename )
    %UNTITLED2 Read and send configuration file to device over serial port
    %   Detailed explanation goes here
    
    text  = fileread(filename);
    lines = strsplit(text,'\n');
    
    for i=1:length(lines)
        if strncmp(string(lines{i}), '%', 1)
            fprintf('Ignoring string: %s\n', string(lines{i}));
        else
            fprintf('Sent: %s\n',string(lines{i}));
            fprintf(serial, lines{i});
            out = fgets(serial);
            fprintf('Recieved: %s\n',out);
        end
    end
    disp('Done');
    
end

