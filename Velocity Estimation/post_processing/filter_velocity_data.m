%% Header

%%% Filename:   filter_velocity_data.m
%%% Author:     Carl Stahoviak
%%% Created:    05/01/2019  

clear;
clc;
close all;

format compact
load('colors.mat');

%% Load Vicon System Data

% vehicle = 'jackal';
vehicle = 'quad';

if strcmp(vehicle,'quad')
    path = '/home/carl/Data/subT/Fleming/3d_velEstimation_2019-05-17/';
elseif strcmp(vehicle,'jackal')
    path = '/home/carl/Data/subT/radar-rig/vicon_2019-05-08/';
else
    path = '';
end
filename = 'cfar-800_10Hz_run0';
filetype = '.mat';

mat_file = strcat(path,'/mat_files/',filename,filetype);
load(mat_file);

% NOTE: Vicon pose message orientation in N.W.U. frame
orientation = [pose_orientation_w, ... 
               pose_orientation_x, ... 
               pose_orientation_y, ...
               pose_orientation_z];

twist_linear = [twist_linear_x, twist_linear_y, twist_linear_z];
twist_linear_body = getBodyFrameVelocities( twist_linear, orientation, ...
    pose_time_stamp, twist_time_stamp );

% Map from NWU to NED coordinate frame
twist_linear_body(:,2) = -twist_linear_body(:,2);
twist_linear_body(:,3) = -twist_linear_body(:,3);

% pose time stamp difference
pose_stamp_diff = pose_time_stamp(2:end) - pose_time_stamp(1:end-1);
twist_stamp_diff = twist_time_stamp(2:end) - twist_time_stamp(1:end-1);

figure(1)
plot(pose_stamp_diff); hold on
plot(twist_stamp_diff);
xlabel('index','Interpreter','latex');
ylabel('inter-message time','Interpreter','latex');
title('Time between Vicon Pose messages','Interpreter','latex');
 

%% Compute Ground Truth Velocity via Central Difference

% use central difference method to calculate derivative
h = 0.01;   % vrpn system @ 100 Hz
velocity_x = central_diff(pose_position_x, h);
velocity_y = central_diff(pose_position_y, h);
velocity_z = central_diff(pose_position_z, h);
velocity = [velocity_x, velocity_y, velocity_z];

velocity_time_stamp = pose_time_stamp(2:end-1);
velocity_time_second = pose_time_second(2:end-1);

velocity_body = getBodyFrameVelocities( velocity, orientation, ...
        pose_time_stamp, velocity_time_stamp );
    
% Map from NWU to NED coordinate frame
velocity_body(:,2) = -velocity_body(:,2);
velocity_body(:,3) = -velocity_body(:,3);
    
%% Plot Body-Frame Velocity

figure(2)
subplot(3,1,1);
plot(twist_time_second,twist_linear_body(:,1)); hold on
plot(velocity_time_second,velocity_body(:,1))
title('Ground Truth Body-Frame Velocity','Interpreter','latex');
ylabel('$v_x$ [m/s]','Interpreter','latex');
subplot(3,1,2);
plot(twist_time_second,twist_linear_body(:,2)); hold on
plot(velocity_time_second,velocity_body(:,2));
ylabel('$v_y$ [m/s]','Interpreter','latex');
subplot(3,1,3);
plot(twist_time_second,twist_linear_body(:,3)); hold on
plot(velocity_time_second,velocity_body(:,3));
ylabel('$v_z$ [m/s]','Interpreter','latex');
xlabel('time [m.s]','Interpreter','latex');
hdl = legend('Vicon system','central difference');
set(hdl,'Interpreter','latex');

%% Plot Frequency Content of velocity data

n = size(velocity_body,1);
sample_freq = 100;     % vrpn system @ 100 Hz
velocity_freq = fft(velocity_body);
power = abs(velocity_freq).^2/n;
f = (0:n-1)*(sample_freq/n);

figure(3);
subplot(3,1,1); plot(f,power(:,1));
subplot(3,1,2); plot(f,power(:,2));
subplot(3,1,3); plot(f,power(:,3));

%% Use lowpass filter to smooth velocity data

fpass = 0.12;
velocity_body_lowpass1 = lowpass(velocity_body,fpass,sample_freq);

wpass = fpass;
velocity_body_lowpass2 = lowpass(velocity_body,wpass);

figure(4)
subplot(3,1,1);
% plot(twist_time_second,twist_linear_body(:,1)); hold on
plot(velocity_time_second,velocity_body_lowpass1(:,1))
title('Ground Truth Body-Frame Velocity - Low-Pass Filtered (1)','Interpreter','latex');
ylabel('$v_x$ [m/s]','Interpreter','latex');
xlim([0, velocity_time_second(end)]);
subplot(3,1,2);
% plot(twist_time_second,twist_linear_body(:,2)); hold on
plot(velocity_time_second,velocity_body_lowpass1(:,2));
ylabel('$v_y$ [m/s]','Interpreter','latex');
xlim([0, velocity_time_second(end)]);
subplot(3,1,3);
% plot(twist_time_second,twist_linear_body(:,3)); hold on
plot(velocity_time_second,velocity_body_lowpass1(:,3));
ylabel('$v_z$ [m/s]','Interpreter','latex');
xlabel('time [m.s]','Interpreter','latex');
xlim([0, velocity_time_second(end)]);
% hdl = legend('Vicon system','central difference');
% set(hdl,'Interpreter','latex');

figure(5)
subplot(3,1,1);
% plot(twist_time_second,twist_linear_body(:,1)); hold on
plot(velocity_time_second,velocity_body_lowpass2(:,1))
title('Ground Truth Body-Frame Velocity - Low-Pass Filtered (2)','Interpreter','latex');
ylabel('$v_x$ [m/s]','Interpreter','latex');
xlim([0, velocity_time_second(end)]);
subplot(3,1,2);
% plot(twist_time_second,twist_linear_body(:,2)); hold on
plot(velocity_time_second,velocity_body_lowpass2(:,2));
ylabel('$v_y$ [m/s]','Interpreter','latex');
xlim([0, velocity_time_second(end)]);
subplot(3,1,3);
% plot(twist_time_second,twist_linear_body(:,3)); hold on
plot(velocity_time_second,velocity_body_lowpass2(:,3));
ylabel('$v_z$ [m/s]','Interpreter','latex');
xlabel('time [m.s]','Interpreter','latex');
xlim([0, velocity_time_second(end)]);
% hdl = legend('Vicon system','central difference');
% set(hdl,'Interpreter','latex');

%% find the "spikes" in the velocity data

deriv =  abs(diff(vecnorm(twist_linear_body,2,2)));

% find the "spikes" in the velocity data
diff_vx = abs(diff(twist_linear_body(:,1)));
diff_vy = abs(diff(twist_linear_body(:,2)));
diff_vz = abs(diff(twist_linear_body(:,3)));

figure(6)
subplot(3,1,1); plot(twist_time_second(1:end-1),diff_vx)
xlim([0 twist_time_second(end-1)]);
subplot(3,1,2); plot(twist_time_second(1:end-1),diff_vy)
xlim([0 twist_time_second(end-1)]);
subplot(3,1,3); plot(twist_time_second(1:end-1),diff_vz)
xlim([0 twist_time_second(end-1)]);

figure(7)
subplot(4,1,1);
plot(twist_time_second(1:end-1),deriv)
xlim([0 twist_time_second(end-1)]);
subplot(4,1,2);
plot(twist_time_second,twist_linear_body(:,1),...
    'color',colors(2,:),'LineWidth',1);
xlim([0 twist_time_second(end)]);
subplot(4,1,3);
plot(twist_time_second,twist_linear_body(:,2),...
    'color',colors(2,:),'LineWidth',1); hold on
xlim([0 twist_time_second(end)]);
subplot(4,1,4);
plot(twist_time_second,twist_linear_body(:,3),...
    'color',colors(2,:),'LineWidth',1); hold on
xlim([0 twist_time_second(end)]);

%% Implement Matlab's smooth() function - Moving Average filter

span = 5;
method = 'moving';
title_base = 'Moving Average Filter - Span = ';

smoothed_vx = smooth(twist_linear_body(:,1),span,method);
smoothed_vy = smooth(twist_linear_body(:,2),span,method);
smoothed_vz = smooth(twist_linear_body(:,3),span,method);

figure(8)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 10;
smoothed_vx = smooth(twist_linear_body(:,1),span,method);
smoothed_vy = smooth(twist_linear_body(:,2),span,method);
smoothed_vz = smooth(twist_linear_body(:,3),span,method);

figure(9)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 15;
smoothed_vx = smooth(twist_linear_body(:,1),span,method);
smoothed_vy = smooth(twist_linear_body(:,2),span,method);
smoothed_vz = smooth(twist_linear_body(:,3),span,method);

figure(10)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 20;
smoothed_vx = smooth(twist_linear_body(:,1),span,method);
smoothed_vy = smooth(twist_linear_body(:,2),span,method);
smoothed_vz = smooth(twist_linear_body(:,3),span,method);

figure(11)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

%% Implement Matlab's smooth() function - Savitzky-Golay filter

span = 5;
degree = 3;
method = 'sgolay';
title_base = 'Savitzky-Golay - Span = ';

smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(12)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 10;
smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(13)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 15;
smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(14)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 20;
smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(15)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

%% Implement Matlab's smooth() function - rloess filter

span = 5;
method = 'rloess';
title_base = 'R-LOESS - Span = ';

smoothed_vx = smooth(twist_linear_body(:,1),span,method);
smoothed_vy = smooth(twist_linear_body(:,2),span,method);
smoothed_vz = smooth(twist_linear_body(:,3),span,method);

figure(16)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 10;
smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(17)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 15;
smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(18)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

span = 20;
smoothed_vx = smooth(twist_linear_body(:,1),span,method,degree);
smoothed_vy = smooth(twist_linear_body(:,2),span,method,degree);
smoothed_vz = smooth(twist_linear_body(:,3),span,method,degree);

figure(19)
subplot(3,1,1); plot(twist_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 twist_time_second(end)]);
subplot(3,1,2); plot(twist_time_second,smoothed_vy);
xlim([0 twist_time_second(end)]);
subplot(3,1,3); plot(twist_time_second,smoothed_vz);
xlim([0 twist_time_second(end)]);

%% Apply rloess filter to central diff + lowpass filter method

title_base = 'Central Diff + Lowpass Filter - RLOESS - Span = ';
method = 'rloess';

span = 5;
smoothed_vx = smooth(velocity_body_lowpass1(:,1),span,method,degree);
smoothed_vy = smooth(velocity_body_lowpass1(:,2),span,method,degree);
smoothed_vz = smooth(velocity_body_lowpass1(:,3),span,method,degree);

figure(20)
subplot(3,1,1); plot(velocity_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 velocity_time_second(end)]);
subplot(3,1,2); plot(velocity_time_second,smoothed_vy);
xlim([0 velocity_time_second(end)]);
subplot(3,1,3); plot(velocity_time_second,smoothed_vz);
xlim([0 velocity_time_second(end)]);

span = 10;
smoothed_vx = smooth(velocity_body_lowpass1(:,1),span,method,degree);
smoothed_vy = smooth(velocity_body_lowpass1(:,2),span,method,degree);
smoothed_vz = smooth(velocity_body_lowpass1(:,3),span,method,degree);

figure(21)
subplot(3,1,1); plot(velocity_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 velocity_time_second(end)]);
subplot(3,1,2); plot(velocity_time_second,smoothed_vy);
xlim([0 velocity_time_second(end)]);
subplot(3,1,3); plot(velocity_time_second,smoothed_vz);
xlim([0 velocity_time_second(end)]);

span = 15;
smoothed_vx = smooth(velocity_body_lowpass1(:,1),span,method,degree);
smoothed_vy = smooth(velocity_body_lowpass1(:,2),span,method,degree);
smoothed_vz = smooth(velocity_body_lowpass1(:,3),span,method,degree);

figure(22)
subplot(3,1,1); plot(velocity_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 velocity_time_second(end)]);
subplot(3,1,2); plot(velocity_time_second,smoothed_vy);
xlim([0 velocity_time_second(end)]);
subplot(3,1,3); plot(velocity_time_second,smoothed_vz);
xlim([0 velocity_time_second(end)]);

span = 20;
smoothed_vx = smooth(velocity_body_lowpass1(:,1),span,method,degree);
smoothed_vy = smooth(velocity_body_lowpass1(:,2),span,method,degree);
smoothed_vz = smooth(velocity_body_lowpass1(:,3),span,method,degree);

figure(23)
subplot(3,1,1); plot(velocity_time_second,smoothed_vx);
title(strcat(title_base,num2str(span)),'Interpreter','latex');
xlim([0 velocity_time_second(end)]);
subplot(3,1,2); plot(velocity_time_second,smoothed_vy);
xlim([0 velocity_time_second(end)]);
subplot(3,1,3); plot(velocity_time_second,smoothed_vz);
xlim([0 velocity_time_second(end)]);
