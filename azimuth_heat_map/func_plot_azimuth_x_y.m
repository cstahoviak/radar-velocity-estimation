% this program is going to take in the raw azimuth heat map data
% from the binary file, and then plot it in x,y space
% inted to be used after loading in a binary file from cloud demo
% using the main_read_demo_file_all_packets

range_Doppler_FFT = static_azimuth_heat_map;

angle_Npts = 64;
N_range = 256/2;
N_Tx = 2;
N_Rx = 4;
range_resolution = 0.044;
range_width = 5;
range_depth = 10;
num_pts = 100; % for plot



distance_each_range_gate  = (0:(N_range*2)-1) .* (range_resolution);
range_angle_range = distance_each_range_gate;
% The range-angle_FFT is performed on each chirp
range_angle_heatmap    = ones(N_range,angle_Npts) .* NaN;

%% FFT
% process each range gate
for i = 1:256
    % pre-define the virtual antenna values
    virtual_antenna_complex_value    = ones(8,1) .* NaN;
    

    % for each Tx
    for j = 1:8
     % process each Receiver
        % get the complex values
        virtual_antenna_complex_value(j)   = range_Doppler_FFT(i,j);
    end

    % Get the real and imaginary parts
    I_input  = real(virtual_antenna_complex_value);
    Q_input  = imag(virtual_antenna_complex_value);

    [angle_FFT_power, angle_of_arrival_deg] = func_calc_angle_FFT(I_input, Q_input, angle_Npts);


    range_angle_heatmap(i,:)  = angle_FFT_power;
    range_angle_deg                  = angle_of_arrival_deg;

end

%% Interpolate
% this is not correct yet
% range_angle_heatmap is in r theta, convert to x,y

% actual values of bins
range = distance_each_range_gate;
theta = asin(linspace(-(angle_Npts/2) + 1,(angle_Npts/2) - 1, angle_Npts).*2/angle_Npts);
% in x-y
posX = range'*sin(theta);
posY = range'*cos(theta);

% 100x100 axis (in m) for heat map
xlin = linspace(-range_width, range_width, 100);
ylin = linspace(0, range_depth, 100);

[X,Y] = meshgrid(xlin,ylin);

range_angle_modified = fliplr(range_angle_heatmap);

V = griddata(reshape(posX.',1,[]),reshape(posY.',1,[]),reshape(range_angle_heatmap.',1,[]),X,Y);


%% Plot
d_distance  = range_angle_range(6) - range_angle_range(5);
d_angle     = range_angle_deg(6) - range_angle_deg(5);

% pcolor(range_angle_deg - d_angle/2,range_angle_range - d_distance/2,(range_angle_heatmap));
pcolor(xlin, ylin, V)


