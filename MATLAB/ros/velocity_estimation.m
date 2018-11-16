%% Header

clear;
clc;
close all;

%% LOAD DATA

% load radar data
load('/home/carl/Data/subT/vicon_velocity_estimate_110818/1642/mat_files/linear_best_velocity_res_pkgrp_doppler.mat')


%% CREATE PLOT

% define ? figure
fig1 = figure(1);
ax1 = axes('Parent',fig1);
hold on;
grid on;

% define ? figure
fig2 = figure(2);
ax2 = axes('Parent',fig2);
hold on;
grid on;

% define marker size
sz = 3;

%%

for i=1:size(radar_doppler,1)

    % extract all non-NaN values from row
    idx_nan = ~isnan(radar_doppler(i,:));
    doppler_nonNaN = radar_doppler(i,idx_nan);
    
    % now extract all non-zero values
    idx_zero = (radar_doppler(i,:) ~= 0);
    doppler_nonZero = radar_doppler(i,idx_zero);
    
    % now extract all non-NaN AND non-zero values
    idx = idx_nan & idx_zero;
    doppler = radar_doppler(i,idx);

    % create time vectors
    t_nonNaN  = radar_time_second(i,1)*ones(1,size(doppler_nonNaN,2));
    t_nonZero = radar_time_second(i,1)*ones(1,size(doppler,2));

    % plot all non-NaN data at each time step
    scatter(ax1,t_nonNaN,doppler_nonNaN,sz,'b','filled');
    
    % get velocity components (in Quad frame)
    [ v_x, v_y ] = getLinearVelocity( radar_x(i,idx_nan), radar_y(i,idx_nan), radar_doppler(i,idx_nan) );
    v_x_nonZero = v_x((v_x ~= 0));
    v_x_nonZero_avg(i,1) = mean(v_x_nonZero);
    
    v_y_nonZero = v_y((v_y ~= 0));
    v_y_nonZero_avg(i,1) = mean(v_y_nonZero);
    
    % plot forward velocity of Quad
    hi = scatter(ax2,t_nonZero,v_x_nonZero,sz,'b','filled');  
    h(i) = hi(1);
    
end

% plot average non-zero, non-NaN mean-forward-velocity estimate
h(end+1) = plot(ax2,radar_time_second,v_x_nonZero_avg,'r-','LineWidth',2);
xlim([0 twist_time_second(end)]);
h1 = title({'Point Target Doppler Velocity','1642 - best\_range\_res'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('forward velocity [m/s]','Interpreter','latex');
h1 = legend([h(1),h(end)],{'$v_x$ - Doppler','Mean Doppler'});
set(h1,'Interpreter','latex','Location','best');

% figure(3)
% plot(radar_time_second,v_x_nonZero_avg,'b-'); hold on; grid on;
% plot(radar_time_second,v_y_nonZero_avg,'r-');
% scatter(twist_time_second,twist_linear_x,sz,'b','filled');
% scatter(twist_time_second,twist_linear_y,sz,'r','filled');
% % ylim([-2 4]); 
% xlim([0 twist_time_second(end)]);

figure(4)
plot(radar_time_second,v_x_nonZero_avg,'b--'); hold on; grid on;
plot(radar_time_second,v_y_nonZero_avg,'r--');
plot(twist_time_second,twist_linear_x,'b');
plot(twist_time_second,twist_linear_y,'r');
ylim([-2 4]); xlim([0 twist_time_second(end)]);
h1 = title({'Mean Radar Doppler Velocity Estimate','1642 - best\_range\_res'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('velocity component [m/s]','Interpreter','latex');
h1 = legend('$v_x$ - Doppler','$v_y$ - Doppler','$v_x$ Vicon','$v_y$ Vicon');
set(h1,'Interpreter','latex','Location','best');


function [ v_x, v_y ] = getLinearVelocity( x, y, doppler )

    theta =  atan2(y,x);
    v_x = -doppler.*cos(theta);
    v_y = -doppler.*sin(theta);

end