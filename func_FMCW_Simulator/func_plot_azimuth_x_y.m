% this program is going to take in the raw azimuth heat map data
% from the binary file, and then plot it in x,y space
% inted to be used after loading in a binary file from cloud demo
% using the main_read_demo_file_all_packets

range_Doppler_FFT = static_azimuth_heat_map;

angle_Npts                       = 64;
N_range = 256/2;
N_Tx = 2;
N_Rx = 4;

range_resolution = 0.044;
max_range = 9.02;
distance_each_range_gate  = (0:(N_range*2)-1) .* (range_resolution);
range_angle_range = distance_each_range_gate;
% The range-angle_FFT is performed on each chirp
range_angle_heatmap    = ones(N_range,angle_Npts) .* NaN;

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

% this is not correct yet
% range_angle_heatmap is in r theta, convert to x,y
xlim = zeros(1,256);
ylim = zeros(1,256);
x_y_heat = zeros(256);

range_angle_deg = (range_angle_deg - 90).* -1;
for i=1:64
    theta = deg2rad(range_angle_deg(i));
    for j = 1:256
       r = range_angle_range(j);
       x = r*cos(theta);
       y = r*sin(theta);
       
       xlim(j) = x;
       ylim(j) = y;
        
       x_y_heat(int8(x),int8(y)) = range_angle_heatmap(j,i);
       
    end
end



d_distance  = range_angle_range(6) - range_angle_range(5);
d_angle     = range_angle_deg(6) - range_angle_deg(5);

pcolor(range_angle_deg - d_angle/2,range_angle_range - d_distance/2,(range_angle_heatmap));


