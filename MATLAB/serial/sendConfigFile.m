function [rangeRes, maxRange, radialVelRes, maxRadialVel] = sendConfigFile( serial,filename)
    %UNTITLED2 Read and send configuration file to device over serial port
    %   Detailed explanation goes here
    
    text  = fileread(filename);
    lines = strsplit(text,'\n');
    
    for i=1:length(lines)
        if strncmp(string(lines{i}), '%', 1)
            %sets the range res, max range, radial velocity res, and max radial velocity
            if strncmp(string(lines{i}), '% Range Resolution', 18)
               [~,remain] = strtok(string(lines{i}), ':');
               remain = strip(remain, 'left', ':');
               rangeRes = str2double(remain);
               fprintf('Set Range Resolution: %f\n', rangeRes);
            elseif strncmp(string(lines{i}), '% Maximum unambiguous Range', 27)
               [~, remain] = strtok(string(lines{i}),':');
               remain = strip(remain,'left',':');
               maxRange = str2double(remain);
               fprintf('Set Max Range: %f\n', maxRange);
            elseif strncmp(string(lines{i}), '% Radial velocity resolution', 28)
               [~, remain] = strtok(string(lines{i}),':');
               remain = strip(remain,'left',':');
               radialVelRes = str2double(remain);
               fprintf('Set Radial Velocity Resolution: %f\n', radialVelRes);
            elseif strncmp(string(lines{i}), '% Maximum Radial Velocity', 25)
               [~, remain] = strtok(string(lines{i}),':');
               remain = strip(remain,'left',':');
               maxRadialVel = str2double(remain);
               fprintf('Set Max Radial Velocity: %f\n', maxRadialVel);
            else
                fprintf('Ignoring string: %s\n', string(lines{i}));
            end
        else
            
            fprintf('Sent: %s\n',string(lines{i}));
            fprintf(serial, lines{i});
            
            out = fgets(serial);
            fprintf('Recieved: %s',out);
            
            out = fgets(serial);
            fprintf('Recieved: %s\n',out);
            
        end
    end  
end

