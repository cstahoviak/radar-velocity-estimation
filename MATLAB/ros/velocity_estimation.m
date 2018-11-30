%% Header

clear;
clc;
close all;

%% LOAD DATA

% load radar data
load('/home/carl/Data/subT/vicon_velocity_estimate_110818/1642/mat_files/zigzag_best_velocity_res_pkgrp_doppler.mat')

% undo RVIZ plotting defaults
radar_y = -radar_y;

radar_angle = atan(radar_y./radar_x);

%% CREATE PLOTS

plot_polar = false;

fig1 = figure(1);
ax1 = axes('Parent',fig1);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title('Point Target Doppler Velocity - All Data Points');
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler Velocity [m/s]','Interpreter','latex');

% Point target doppler velocity figure
fig2 = figure(2);
ax2 = axes('Parent',fig2);
hold on; grid on;
h1 = title(ax2,{'Point Target Doppler Velocity','1443 - zigzag\_best\_velocity\_res\_pkgrp\_doppler'});
set(h1,'Interpreter','latex');
xlabel(ax2,'time [s]','Interpreter','latex');
ylabel(ax2,'doppler velocity [m/s]','Interpreter','latex');

% Point target intensity figure
fig4 = figure(4);
ax4 = axes('Parent',fig4);
hold on; grid on;
h1 = title('Point Target Intensity Value');
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Intensity [dB]','Interpreter','latex');
xlim([0 radar_time_second(end)]);

% Range threshold figure
fig5 = figure(5);
ax5 = axes('Parent',fig5);
hold on; grid on;
h1 = title('Target Range Threshold');
set(h1,'Interpreter','latex');
xlabel('y [m]','Interpreter','latex');
ylabel('x [m]','Interpreter','latex');

% (r,theta) polar plot
if plot_polar
    fig6 = figure(6);
    ax6 = polaraxes('Parent',fig6);
    hold on; grid on;
    h1 = title('Point Target Location - Polar Coordinates');
    set(h1,'Interpreter','latex');
    thetalim(ax6,[-135 135]);
    ax6.ThetaZeroLocation = 'top';
    ax6.ThetaDir = 'clockwise';
end

% color doppler data by angle off boresight
fig7 = figure(7);
ax7 = axes('Parent',fig7);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title('Point Target Doppler Velocity - By Angle');
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler Velocity [m/s]','Interpreter','latex');
h = colorbar(ax7); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax7,'jet')

% apply angle filtering to doppler data
fig8 = figure(8);
ax8 = axes('Parent',fig8);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title('Point Target Doppler Velocity - Angle Filtering');
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');

% Point target doppler velocity figure
fig10 = figure(10);
ax10 = axes('Parent',fig10);
hold on; grid on;
h1 = title(ax10,{'Point Target Doppler Velocity - Forward','1443 - zigzag\_best\_velocity\_res\_pkgrp\_doppler'});
set(h1,'Interpreter','latex');
xlabel(ax2,'time [s]','Interpreter','latex');
ylabel(ax2,'forward velocity [m/s]','Interpreter','latex');

% Point target doppler velocity figure
fig11 = figure(11);
ax11 = axes('Parent',fig11);
hold on; grid on;
h1 = title(ax11,{'Point Target Doppler Velocity - Lateral','1443 - zigzag\_best\_velocity\_res\_pkgrp\_doppler'});
set(h1,'Interpreter','latex');
xlabel(ax2,'time [s]','Interpreter','latex');
ylabel(ax2,'lateral velocity [m/s]','Interpreter','latex');

% define marker size
sz = 3;

%% PRELININARY ANALYSIS

% filtering thresholds
range_thres     = 0.3;              % [m]
angle_thres     = deg2rad(0.1);       % [rad]
intensity_thres = 15;               % [dB]

for i=1:size(radar_doppler,1)

    % extract all non-NaN values from row
    idx_nonNaN = ~isnan(radar_doppler(i,:));
    doppler_nonNaN = radar_doppler(i,idx_nonNaN);
    
    % now extract all non-zero values (will NOT also remove NaN values)
    idx_nonZero = (radar_doppler(i,:) ~= 0);
    doppler_nonZero = radar_doppler(i,idx_nonZero);
    
    % now extract all non-NaN AND non-zero values
    idx = idx_nonNaN & idx_nonZero;
    doppler = radar_doppler(i,idx);
    
    % extract index of points with zero Doppler velocity
    idx_ISzero = (radar_doppler(i,:) == 0);

    % create time vectors
    t_nonNaN  = radar_time_second(i,1)*ones(1,size(doppler_nonNaN,2));
    t = radar_time_second(i,1)*ones(1,size(doppler,2));
    t_ISZero  = radar_time_second(i,1)*ones(1,sum(idx_ISzero,2)); 

    % plot all non-NaN data at each time step
    scatter(ax1,t_nonNaN,-doppler_nonNaN,sz,'b','filled');
    
    % get velocity components (in Quad frame)
%     [ v_x, v_y ] = getVelocity( radar_doppler(i,idx_nonNaN), radar_angle(i,idx_nonNaN) );
%     v_x_nonZero = v_x((v_x ~= 0));
%     v_x_nonZero_avg(i,1) = mean(v_x_nonZero);
%     
%     v_y_nonZero = v_y((v_y ~= 0));
%     v_y_nonZero_avg(i,1) = mean(v_y_nonZero);
    
    % plot (non-zero) forward velocity of Quad
%     hi = scatter(ax2,t,v_x_nonZero,sz,'b','filled');
    hi = scatter(ax2,t,-doppler,sz,'b','filled'); 
    h(i) = hi(1);
    
    % plot (non-zero doppler) intensity values at each time step
    scatter(ax4,t,radar_intensity(i,idx),sz,'b','filled');
    
    % filter intensity by zero Doppler velocity
    % NOTE: interesting horizantal line trend!
    scatter(ax4,t_ISZero,radar_intensity(i,idx_ISzero),sz,'r','filled');
    
    % filter (x,y) points by zero Doppler velocity
    idx_range = (radar_range(i,:) > range_thres);
    scatter(ax5,radar_y(i,idx),radar_x(i,idx),sz,'b','filled');
    scatter(ax5,radar_y(i,idx_ISzero),radar_x(i,idx_ISzero),sz,'r','filled');
    
    if plot_polar
        polarscatter(ax6,radar_angle(i,idx_range),radar_range(i,idx_range),sz,'b','filled');
        polarscatter(ax6,radar_angle(i,~idx_range),radar_range(i,~idx_range),sz,'r','filled');
    end
    
    % apply color gradient based on target angle from boresight
    scatter(ax7,t,-doppler,8,rad2deg(abs(radar_angle(i,idx))),'filled');
    
    % apply angle filtering to doppler data... not working!
    idx_angle = (radar_angle(i,idx) < angle_thres);
    doppler_filter_angle = radar_doppler(i,idx_angle);
    t_angle = radar_time_second(i,1)*ones(1,size(doppler_filter_angle,2));
    scatter(ax8,t_angle,-doppler_filter_angle,sz,'b','filled');
    
    % apply range filtering to doppler data
    
    % apply intensity filtering to doppler data
    
    % apply (range,angle,intensity) filtering to doppler data
    
    % Scale doppler velocity by angle - forward estimate
    hi = scatter(ax10,t,-doppler.*(1./cos(radar_angle(i,idx))),sz,'b','filled'); 
    h_fwd(i) = hi(1);
    
    % Scale doppler velocity by angle - lateral estimate
    hi = scatter(ax11,t,-doppler.*(1./sin(radar_angle(i,idx))),sz,'b','filled');
    h_lat(i) = hi(1);
    
    
end

% add Vicon truth data for v_x and v_y to plot
h(end+1) = plot(ax2,twist_time_second,twist_linear_x,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
h(end+1) = plot(ax2,twist_time_second,twist_linear_y,'color',[0.6350, 0.0780, 0.1840],'LineWidth',1);
ylim(ax2,[-3 3]); xlim(ax2,[0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend([h(1),h(end-1),h(end)],{'Radar Doppler','$v_x$ - Vicon','$v_y$ - Vicon'});
set(h1,'Interpreter','latex','Location','best');

% add Vicon truth data for v_x for scaled forward velocity plot
h_fwd(end+1) = plot(ax10,twist_time_second,twist_linear_x,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
ylim(ax10,[-3 3]); xlim(ax10,[0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend([h_fwd(1),h_fwd(end)],{'Radar Doppler','$v_x$ - Vicon'});
set(h1,'Interpreter','latex','Location','best');

% add Vicon truth data for v_x for scaled lateral velocity plot
h_lat(end+1) = plot(ax11,twist_time_second,twist_linear_y,'color',[0.6350, 0.0780, 0.1840],'LineWidth',1);
ylim(ax11,[-3 3]); xlim(ax11,[0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend([h_lat(1),h_lat(end)],{'Radar Doppler','$v_y$ - Vicon'});
set(h1,'Interpreter','latex','Location','best');

% set figure 4 legend
h1 = legend([h4_1(1),h4_2(1)],'non-zero Doppler','zero Doppler');
set(h1,'Interpreter','latex','Location','best');

%% SECONDARY ANALYSIS

% Point Filtering Ideas
%   1. Remove targets near sensor origin
%   2. Remove targets outside of relaible FOV
%   3. Remove targets below a certain intensity threshold

% Averaging
%   1. Weighted averaging - assign weights by intensity

% for i=1:size(radar_doppler,1)
%     
%     
%     
% end

function [ v_x, v_y ] = getVelocity( doppler,theta )

    v_x = -doppler.*cos(theta);
    v_y = -doppler.*sin(theta);

end