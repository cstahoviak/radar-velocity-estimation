%% Header

clear;
clc;
close all;

%% LOAD DATA

% load radar data
load('/home/carl/Data/subT/vicon_velocity_estimate_110818/1642/mat_files/linear_best_velocity_res_pkgrp_doppler.mat')

% undo RVIZ plotting defaults
radar_y = -radar_y;

% calculate radar angle
radar_angle = atan(radar_y./radar_x);

%% CREATE PLOTS

plot_polar = false;

fig1 = figure(1);
ax1 = axes('Parent',fig1);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax1, {'Point Target Doppler Velocity','All Data Points'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler Velocity [m/s]','Interpreter','latex');

% Point target doppler velocity figure
fig2 = figure(2);
ax2 = axes('Parent',fig2);
hold on; grid on;
h1 = title(ax2, {'Point Target Doppler Velocity','Non-Zero Doppler Data Points'});
set(h1,'Interpreter','latex');
xlabel(ax2,'time [s]','Interpreter','latex');
ylabel(ax2,'doppler velocity [m/s]','Interpreter','latex');
ylim(ax2,[-2.5 2.5]); xlim(ax2,[0 max([radar_time_second(end) twist_time_second(end)])]);

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
h1 = title(ax7, {'Point Target Doppler Velocity','Angle from Boresight'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler Velocity [m/s]','Interpreter','latex');
h = colorbar(ax7); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax7,'jet')
caxis([0 75])

% apply ANGLE filtering to doppler data
fig8 = figure(8);
ax8 = axes('Parent',fig8);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax8, {'Point Target Doppler Velocity','Filtered by \textbf{ANGLE}'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
h = colorbar(ax8); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax8,'jet')
caxis([0 75])

% apply RANGE filtering to doppler data
fig9 = figure(9);
ax9 = axes('Parent',fig9);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax9, {'Point Target Doppler Velocity','Filtered by \textbf{RANGE}'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
h = colorbar(ax9); ylabel(h,'Range [m]','Interpreter','latex')
colormap(ax9,'jet')

% apply INTENSITY filtering to doppler data
fig10 = figure(10);
ax10 = axes('Parent',fig10);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax10, {'Point Target Doppler Velocity','Filtered by \textbf{INTENSITY}'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
h = colorbar(ax10); ylabel(h,'Intensity [dB]','Interpreter','latex')
colormap(ax10,'jet')

% apply ANGLE, RANGE, INTENSITY filtering to doppler data
fig11 = figure(11);
ax11 = axes('Parent',fig11);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax11, {'Point Target Doppler Velocity','Filtered by \textbf{Angle, Range \& Intensity}'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
h = colorbar(ax11); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax11,'jet')
caxis([0 75])

% Point target doppler velocity figure
fig12 = figure(12);
ax12 = axes('Parent',fig12);
hold on; grid on;
h1 = title(ax12,{'Point Target Doppler Velocity - Forward','Scaled by Angle - $1/\cos(\theta)$'});
set(h1,'Interpreter','latex');
xlabel(ax12,'time [s]','Interpreter','latex');
ylabel(ax12,'forward velocity [m/s]','Interpreter','latex');
h = colorbar(ax12); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax12,'jet')

% Point target doppler velocity figure
fig13 = figure(13);
ax13 = axes('Parent',fig13);
hold on; grid on;
h1 = title(ax13,{'Scaled Point Target Doppler Velocity - Lateral','Scaled by Angle - $1/\sin(\theta)$'});
set(h1,'Interpreter','latex');
xlabel(ax13,'time [s]','Interpreter','latex');
ylabel(ax13,'lateral velocity [m/s]','Interpreter','latex');
h = colorbar(ax13); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax13,'jet')

% Point target doppler velocity figure
fig14 = figure(14);
ax14 = axes('Parent',fig14);
hold on; grid on;
h1 = title(ax14,{'Point Target Doppler Velocity - Forward','\textbf{Range, Angle, Intensity} Filtered'});
set(h1,'Interpreter','latex');
xlabel(ax12,'time [s]','Interpreter','latex');
ylabel(ax12,'forward velocity [m/s]','Interpreter','latex');
% h = colorbar(ax12=4); ylabel(h,'Angle [deg]','Interpreter','latex')
% colormap(ax14,'jet')

% define marker size
sz = 4;

%% PRELININARY ANALYSIS

% filtering thresholds
% range_thres     = 0.3;              % [m]
% angle_thres     = 35;               % [deg]
% intensity_thres = 20;               % [dB]

range_thres     = 0;                % [m]
angle_thres     = 90;               % [deg]
intensity_thres = 0;                % [dB]

for i=1:size(radar_doppler,1)

    % extract all non-NaN values from row
    idx_nonNaN = ~isnan(radar_doppler(i,:));
    doppler_nonNaN = radar_doppler(i,idx_nonNaN);
    
    % now extract all non-zero values (will NOT also remove NaN values)
    idx_nonZero = (radar_doppler(i,:) ~= 0);
    doppler_nonZero = radar_doppler(i,idx_nonZero);
    
    % now extract all non-NaN AND non-zero values
%     idx = idx_nonNaN & idx_nonZero;
    idx = idx_nonNaN;
    doppler = radar_doppler(i,idx);
    
    % extract index of points with zero Doppler velocity
    idx_isZero = (radar_doppler(i,:) == 0);
    
    % range, angle, intensity filtering:
    idx_angle = (abs(rad2deg(radar_angle(i,:))) < angle_thres);
    idx_range = (radar_range(i,:) > range_thres);
    idx_intensity = (radar_intensity(i,:) > intensity_thres);

    % create time vectors
    t_nonNaN  = radar_time_second(i,1)*ones(1,size(doppler_nonNaN,2));
    t = radar_time_second(i,1)*ones(1,size(doppler,2));
    t_isZero  = radar_time_second(i,1)*ones(1,sum(idx_isZero,2)); 

    % plot all non-NaN data at each time step
    scatter(ax1,t,-doppler,sz,'b','filled');
    scatter(ax1,t_isZero,-radar_doppler(i,idx_isZero),sz,'r','filled');
    
    % get velocity components (in Quad frame)
%     [ v_x, v_y ] = getVelocity( radar_doppler(i,idx_nonNaN), radar_angle(i,idx_nonNaN) );
%     v_x_nonZero = v_x((v_x ~= 0));
%     v_x_nonZero_avg(i,1) = mean(v_x_nonZero);
%     
%     v_y_nonZero = v_y((v_y ~= 0));
%     v_y_nonZero_avg(i,1) = mean(v_y_nonZero);
    
    % plot (non-zero) doppler velocity of Quad
    hi = scatter(ax2,t,-doppler,sz,'b','filled'); 
    h(i) = hi(1);
    
    % plot (non-zero doppler) intensity values at each time step
    h4_1 = scatter(ax4,t,radar_intensity(i,idx),sz,'b','filled');
    % filter intensity by zero Doppler velocity, NOTE: interesting horizantal line trend!
    h4_2 = scatter(ax4,t_isZero,radar_intensity(i,idx_isZero),sz,'r','filled');
    
    % filter (x,y) points by zero Doppler velocity
    scatter(ax5,radar_y(i,idx_nonNaN & idx_nonZero),radar_x(i,idx_nonNaN & idx_nonZero),sz,'b','filled');
    scatter(ax5,radar_y(i,idx_isZero),radar_x(i,idx_isZero),sz,'r','filled');
    
    if plot_polar
        polarscatter(ax6,radar_angle(i,idx_range),radar_range(i,idx_range),sz,'b','filled');
        polarscatter(ax6,radar_angle(i,~idx_range),radar_range(i,~idx_range),sz,'r','filled');
    end
    
    % apply color gradient based on target angle from boresight
    color = rad2deg(abs(radar_angle(i,idx)));
    scatter(ax7,t,-doppler,8,color,'filled');
    
    % Scale doppler velocity by angle - forward estimate
    hi = scatter(ax12,t,-doppler.*(1./cos(radar_angle(i,idx))),sz,color,'filled');
    h_fwd(i) = hi(1);
    
    % Scale doppler velocity by angle - lateral estimate
    hi = scatter(ax13,t,-doppler.*(1./sin(radar_angle(i,idx))),sz,color,'filled');
    h_lat(i) = hi(1);
    
    % apply angle filtering to doppler data... not working!
    doppler_filter_angle = radar_doppler(i,idx & idx_angle);
    t_angle = radar_time_second(i,1)*ones(1,size(doppler_filter_angle,2));
    color = rad2deg(abs(radar_angle(i,idx & idx_angle)));
    scatter(ax8,t_angle,-doppler_filter_angle,sz,color,'filled');
    
    % apply range filtering to doppler data
    doppler_filter_range = radar_doppler(i,idx & idx_range);
    t_range = radar_time_second(i,1)*ones(1,size(doppler_filter_range,2));
    color = radar_range(i,idx & idx_range);
    scatter(ax9,t_range,-doppler_filter_range,sz,color,'filled');
    
    % apply intensity filtering to doppler data
    doppler_filter_int = radar_doppler(i,idx & idx_intensity);
    t_int = radar_time_second(i,1)*ones(1,size(doppler_filter_int,2));
    color = radar_intensity(i,idx & idx_intensity);
    scatter(ax10,t_int,-doppler_filter_int,sz,color,'filled');
    
    % apply (angle,range,intensity) filtering to doppler data
    idx_ARI = idx & idx_angle & idx_range & idx_intensity;
    doppler_filter_all = radar_doppler(i,idx_ARI);
    t_all = radar_time_second(i,1)*ones(1,size(doppler_filter_all,2));
    color = rad2deg(abs(radar_angle(i,idx_ARI)));
    scatter(ax11,t_all,-doppler_filter_all,sz,color,'filled');
    
    % Scale doppler velocity by angle - forward estimate
    scatter(ax14,t_all,-doppler_filter_all.*(1./cos(radar_angle(i,idx_ARI))),sz,'b','filled');
    
end

vx_ymax = 3;
vx_ymin = -3;

% add Vicon truth data for v_x and v_y to plot
h(end+1) = plot(ax2,twist_time_second,twist_linear_x,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
% h(end+1) = plot(ax2,twist_time_second,twist_linear_y,'color',[0.6350, 0.0780, 0.1840],'LineWidth',1);
h1 = legend([h(1),h(end-1),h(end)],{'Radar Doppler','$v_x$ - Vicon','$v_y$ - Vicon'});
set(h1,'Interpreter','latex','Location','best');

% set figure 4 legend
h1 = legend([h4_1(1),h4_2(1)],'non-zero Doppler','zero Doppler');
set(h1,'Interpreter','latex','Location','best');

% add Vicon truth data to plots
add_vicon = false;
if add_vicon
    addViconData( ax8, twist_time_second,twist_linear_x )
    addViconData( ax9, twist_time_second,twist_linear_x )
    addViconData( ax10, twist_time_second,twist_linear_x )
    addViconData( ax11, twist_time_second,twist_linear_x )
    addViconData( ax14, twist_time_second,twist_linear_x )
end

% add Vicon truth data for v_x for scaled forward velocity plot
h_fwd(end+1) = plot(ax12,twist_time_second,twist_linear_x,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
ylim(ax12,[vx_ymin vx_ymax]); xlim(ax12,[0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend([h_fwd(1),h_fwd(end)],{'Radar Doppler','$v_x$ - Vicon'});
set(h1,'Interpreter','latex','Location','best');

% add Vicon truth data for v_x for scaled lateral velocity plot
h_lat(end+1) = plot(ax13,twist_time_second,twist_linear_y,'color',[0.6350, 0.0780, 0.1840],'LineWidth',1);
ylim(ax12,[vx_ymin vx_ymax]); xlim(ax13,[0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend([h_lat(1),h_lat(end)],{'Radar Doppler','$v_y$ - Vicon'});
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

function [ ] = addViconData( ax,time,data )

    vx_ymax = 3;
    vx_ymin = -3;

    plot(ax,time,data,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
    ylim(ax,[vx_ymin vx_ymax]); xlim(ax,[0 time(end)]);

end

function [ v_x, v_y ] = getVelocity( doppler,theta )

    v_x = -doppler.*cos(theta);
    v_y = -doppler.*sin(theta);

end