% ReadMe.m
% 11-July-2018
% C.Williams

% This directory contains Matlab files that read a binary file generated by the mmWave Demo Visualizer.

% The main routine is in: main_read_demo_file_all_packets.m

% The separate functions handle different types of TVL data packets.

% A single data file is provided with the extension: *.txt

% Note that the main routine wants the data file in the directory: '..\raw_data\'

% Note that the initial search for the MagicWord may not be necessary for files already created, 
% but we know that some type of searching logic will be needed for reading data directly 
% using the serial port.

% The main routine first reads the whole binary data file to determine the number of packets stored in the file.
% Then, the main routine defines variables in the Matlab workspace, rewinds the file, and reads all of the
% data into the Matlab workspace.

% After reading all of the data, the code just makes a simple plot. 

% The user can now process the data in the Matlab workspace as they wish.
