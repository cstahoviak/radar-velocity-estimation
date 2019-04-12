%% Header

%%% Filename:   velocity_estimation.m
%%% Author:     Carl Stahoviak
%%% Edited:     03/04/2019  

clear;
clc;
close all;

%% LOAD DATA

% load radar data
load('/home/carl/Data/subT/Fleming/single_radar_velocity_estimate_2018_11_18/1642/mat_files/linear_best_velocity_res_pkgrp_doppler.mat')
% load('/home/carl/Data/subT/Fleming/multiradar_2019_02_13/mat_files/multiradar_2019-02-13_rangeRes_0-04_velRes_0-09_loop.mat')
% load('/home/carl/Data/subT/vicon_velocity_estimate_110818/1642/mat_files/linear_best_range_res.mat')
% load('C:\Users\carlc\Google Drive\Boulder\MS Thesis Project\Data\bagfiles\vicon_velocity_estimate_110818\1642\mat_files\linear_best_velocity_res_pkgrp_doppler.mat')

% undo RVIZ plotting defaults
radar_y = -radar_y;

% calculate radar angle
radar_angle = atan(radar_y./radar_x);   % [rad]

%% CREATE PLOTS

plot_polar = true;

% set y-axis limits
vx_ymax = 2.5;
vx_ymin = -2.5;
vx_ylim = [vx_ymin vx_ymax];

% define marker size
sz = 5;

fig1 = figure(1);
ax1 = axes('Parent',fig1);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax1, {'Point Target Doppler Velocity','All Data Points'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('doppler velocity [m/s]','Interpreter','latex');

% Point target doppler velocity figure
fig2 = figure(2);
ax2 = axes('Parent',fig2);
hold on; grid on;
h1 = title(ax2, {'Point Target Doppler Velocity','Non-Zero Doppler Data Points'});
set(h1,'Interpreter','latex');
xlabel(ax2,'time [s]','Interpreter','latex');
ylabel(ax2,'doppler velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);

% Point target intensity figure
fig4 = figure(4);
ax4 = axes('Parent',fig4);
hold on;
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
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
h = colorbar(ax8); ylabel(h,'Angle [deg]','Interpreter','latex')
colormap(ax8,'jet')
caxis([0 rad2deg(max(max(radar_angle)))])

% apply RANGE filtering to doppler data
fig9 = figure(9);
ax9 = axes('Parent',fig9);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax9, {'Point Target Doppler Velocity','Filtered by \textbf{RANGE}'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
h = colorbar(ax9); ylabel(h,'Range [m]','Interpreter','latex')
colormap(ax9,'jet')
caxis([0 max(max(radar_range))])

% apply INTENSITY filtering to doppler data
fig10 = figure(10);
ax10 = axes('Parent',fig10);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax10, {'Point Target Doppler Velocity','Filtered by \textbf{INTENSITY}'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
h = colorbar(ax10); ylabel(h,'Intensity [dB]','Interpreter','latex')
colormap(ax10,'jet')
caxis([min(min(radar_intensity)) max(max(radar_intensity))])

% apply ANGLE, RANGE, INTENSITY filtering to doppler data
fig11 = figure(11);
ax11 = axes('Parent',fig11);
hold on; grid on;
xlim([0 radar_time_second(end)]);
h1 = title(ax11, {'Point Target Doppler Velocity','\textbf{Range, Angle, Intensity} Filtered'});
set(h1,'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('Doppler velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);

% Point target doppler velocity figure
fig12 = figure(12);
ax12 = axes('Parent',fig12);
hold on; grid on;
h1 = title(ax12,{'Point Target Doppler Velocity - Forward','Scaled by Angle - $1/\cos(\theta)$'});
set(h1,'Interpreter','latex');
xlabel(ax12,'time [s]','Interpreter','latex');
ylabel(ax12,'forward velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
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
% h = colorbar(ax13); ylabel(h,'Angle [deg]','Interpreter','latex')
% colormap(ax13,'jet')

% Point target doppler velocity figure
fig14 = figure(14);
ax14 = axes('Parent',fig14);
hold on; grid on;
h1 = title(ax14,{'Point Target Doppler Velocity - Forward','\textbf{Range, Angle, Intensity} Filtered','Scaled by Angle - $1/\cos(\theta)$'});
set(h1,'Interpreter','latex');
xlabel(ax14,'time [s]','Interpreter','latex');
ylabel(ax14,'forward velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
% h = colorbar(ax12=4); ylabel(h,'Angle [deg]','Interpreter','latex')
% colormap(ax14,'jet')

%% PRELININARY ANALYSIS

% filtering thresholds

% range_thres     = 0;                % [m]
% angle_thres     = 90;               % [deg]
% intensity_thres = 0;                % [dB]

range_thres     = 0.5;              % [m]
angle_thres     = 30;               % [deg]
intensity_thres = 23;               % [dB]
thresholds = [angle_thres, range_thres, intensity_thres];

% init estimate vectors
doppler_estimate_mean = zeros(size(radar_doppler,1),1);
doppler_estimate_max = zeros(size(radar_doppler,1),1);

doppler_estimate_mean_err = zeros(size(radar_doppler,1),1);
doppler_estimate_max_err = zeros(size(radar_doppler,1),1);

% for python troubleshooting
idx_array = zeros(5, size(radar_doppler,1));

for i=1:size(radar_doppler,1)

    % extract all non-NaN values from row
    idx_nonNaN = ~isnan(radar_doppler(i,:));
    doppler_nonNaN = radar_doppler(i,idx_nonNaN);
    
    % now extract all non-zero values (will NOT also remove NaN values)
    idx_nonZero = (radar_doppler(i,:) ~= 0);
    doppler_nonZero = radar_doppler(i,idx_nonZero);
    
    % now extract all non-NaN AND non-zero values
    idx = idx_nonNaN & idx_nonZero;
%     idx = idx_nonNaN;                   % include zero-doppler data points              
    doppler = radar_doppler(i,idx);
    
    % extract index of points with zero Doppler velocity
    idx_isZero = (radar_doppler(i,:) == 0);
    
    % range, angle, intensity filtering:
    idx_angle = (abs(rad2deg(radar_angle(i,:))) < angle_thres);
    idx_range = (radar_range(i,:) > range_thres);
    idx_intensity = (radar_intensity(i,:) > intensity_thres);
    
    % store index values for Python troubleshooting
    idx_array(1:4,i) = [sum(idx); sum(idx_angle); sum(idx_range); sum(idx_intensity)];

    % create time vectors
    t_nonNaN   = radar_time_second(i,1)*ones(1,size(doppler_nonNaN,2));
    t_nonZero  = radar_time_second(i,1)*ones(1,size(doppler_nonZero,2));
    t          = radar_time_second(i,1)*ones(1,size(doppler,2));
    t_isZero   = radar_time_second(i,1)*ones(1,sum(idx_isZero,2)); 

    % plot all non-NaN data at each time step
    h1_1 = scatter(ax1,t,-doppler,sz,'b','filled');
    h1_2 = scatter(ax1,t_isZero,-radar_doppler(i,idx_isZero),sz,'r','filled');
    
    % plot (non-zero) doppler velocity of Quad
    h2 = scatter(ax2,t_nonZero,-doppler_nonZero,sz,'b','filled');
    
    % plot (non-zero doppler) intensity values at each time step
    h4_1 = scatter(ax4,t,radar_intensity(i,idx),sz,'b','filled');
    % filter intensity by zero Doppler velocity, NOTE: interesting horizantal line trend!
    h4_2 = scatter(ax4,t_isZero,radar_intensity(i,idx_isZero),sz,'r','filled');
    
    % filter (x,y) points by zero Doppler velocity
    scatter(ax5,radar_y(i,idx_nonNaN & idx_nonZero),radar_x(i,idx_nonNaN & idx_nonZero),sz,'b','filled');
    scatter(ax5,radar_y(i,idx_isZero),radar_x(i,idx_isZero),sz,'r','filled');
    
    if plot_polar
        polarscatter(ax6,radar_angle(i,idx),radar_range(i,idx),sz,'b','filled');
        polarscatter(ax6,radar_angle(i,~idx),radar_range(i,~idx),sz,'r','filled');
    end
    
    % apply color gradient based on target angle from boresight
    color = rad2deg(abs(radar_angle(i,idx)));
    scatter(ax7,t,-doppler,8,color,'filled');
    
    % Scale doppler velocity by angle - forward estimate
    h12 = scatter(ax12,t,-doppler.*(1./cos(radar_angle(i,idx))),sz,'b','filled');
    
    % Scale doppler velocity by angle - lateral estimate
    hi = scatter(ax13,t,-doppler.*(1./sin(radar_angle(i,idx))),sz,'b','filled');
    h_lat(i) = hi(1);
    
    % apply angle filtering to doppler data
    doppler_filter_angle = radar_doppler(i,idx & idx_angle);
    t_angle = radar_time_second(i,1)*ones(1,size(doppler_filter_angle,2));
    color = rad2deg(abs(radar_angle(i,idx & idx_angle)));
    scatter(ax8,t_angle,-doppler_filter_angle,sz+2,color,'filled');
    
    % apply range filtering to doppler data
    doppler_filter_range = radar_doppler(i,idx & idx_range);
    t_range = radar_time_second(i,1)*ones(1,size(doppler_filter_range,2));
    color = radar_range(i,idx & idx_range);
    scatter(ax9,t_range,-doppler_filter_range,sz+2,color,'filled');
    
    % apply intensity filtering to doppler data
    doppler_filter_int = radar_doppler(i,idx & idx_intensity);
    t_int = radar_time_second(i,1)*ones(1,size(doppler_filter_int,2));
    color = radar_intensity(i,idx & idx_intensity);
    scatter(ax10,t_int,-doppler_filter_int,sz+2,color,'filled');
    
    % apply (angle,range,intensity) filtering to doppler data
    idx_ARI = idx & idx_angle & idx_range & idx_intensity;
    idx_array(5,i) = sum(idx_ARI);
%     disp(sum(idx_ARI))
    doppler_filter_all = radar_doppler(i,idx_ARI);
    t_all = radar_time_second(i,1)*ones(1,size(doppler_filter_all,2));
    h11 = scatter(ax11,t_all,-doppler_filter_all,sz,'b','filled');
    
    % Scale doppler velocity by angle - forward estimate
    doppler_scaled = doppler_filter_all.*(1./cos(radar_angle(i,idx_ARI)));
    h14 = scatter(ax14,t_all,-doppler_scaled,sz,'b','filled');
    
    % estimate forward velocity - mean & max
    if isempty( doppler_scaled )
        doppler_estimate_mean(i,1) = 0;
        doppler_estimate_max(i,1)  = 0;
    else
        doppler_estimate_mean(i,1) = mean(doppler_scaled);
    %     disp(sign(mean(doppler_scaled)));
    %     disp(max(abs(doppler_scaled)));
        doppler_estimate_max(i,1) = sign(mean(doppler_scaled)) * max(abs(doppler_scaled));
    end
    
    % calculate error - handles unequal sample rates between radar and Vicon
%     [ d, ix ] = min( abs( x-val ) );    % ix is the "index in 1D array that has closest value to" val
    [~,ix] = min( abs ( twist_time_second - radar_time_second(i) ) );
    doppler_estimate_mean_err(i,1) = abs( -doppler_estimate_mean(i) - twist_linear_x(ix) );
    doppler_estimate_max_err(i,1) = abs( -doppler_estimate_max(i) - twist_linear_x(ix) );
    
end

RMSE_mean = sqrt(mean(doppler_estimate_mean_err.^2))
RMSE_max  = sqrt(mean(doppler_estimate_max_err.^2))

% set figure 1 legend
h1 = legend([h1_1(1),h1_2(1)],'non-zero Doppler','zero Doppler');
set(h1,'Interpreter','latex','Location','best');

% set figure 4 legend
h1 = legend([h4_1(1),h4_2(1)],'non-zero Doppler','zero Doppler');
set(h1,'Interpreter','latex','Location','best');

% add Vicon truth data to plots
add_vicon = true;
if add_vicon
%     addViconData( ax8, twist_time_second,twist_linear_x )
%     addViconData( ax9, twist_time_second,twist_linear_x )
%     addViconData( ax10, twist_time_second,twist_linear_x )
    addViconData( h2(1), ax2, twist_time_second,twist_linear_x )
    addViconData( h11(1), ax11, twist_time_second,twist_linear_x )
    addViconData( h12(1), ax12, twist_time_second,twist_linear_x )
    addViconData( h14(1), ax14, twist_time_second,twist_linear_x )
end 

% add Vicon truth data for v_y for scaled lateral velocity plot
h_lat(end+1) = plot(ax13,twist_time_second,twist_linear_y,'color',[0.6350, 0.0780, 0.1840],'LineWidth',1);
ylim(ax12,[vx_ymin vx_ymax]); xlim(ax13,[0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend([h_lat(1),h_lat(end)],{'Radar Doppler','$v_y$ - Vicon'});
set(h1,'Interpreter','latex','Location','best');

% plot velocity estimate
fig15 = figure(15);
ax15 = axes('Parent',fig15);
hold on; grid on;
plot(ax15,twist_time_second,twist_linear_x,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
plot(ax15,radar_time_second,-doppler_estimate_mean,'color',[0, 0.4470, 0.7410],'LineWidth',1);
plot(ax15,radar_time_second,-doppler_estimate_max,'color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
h1 = title(ax15, {'Doppler Velocity Estimation','Mean and Maximum Estimation'});
set(h1,'Interpreter','latex');
xlabel(ax15,'time [s]','Interpreter','latex');
ylabel(ax15,'forward velocity [m/s]','Interpreter','latex');
ylim([vx_ymin vx_ymax]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend('$v_x$ Vicon','mean estimate','max estimate');
set(h1,'Interpreter','latex','Location','best');

% plot velocity estimate error
fig16 = figure(16);
ax16 = axes('Parent',fig16);
hold on; grid on;
plot(ax16,radar_time_second,doppler_estimate_mean_err,'color',[0, 0.4470, 0.7410],'LineWidth',1);
plot(ax16,radar_time_second,doppler_estimate_max_err,'color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
h1 = title(ax16,'Doppler Velocity Estimation Error');
set(h1,'Interpreter','latex');
str = {strcat('$RMSE_{mean}$ = ',num2str(RMSE_mean),' m/s'),strcat('$RMSE_{max}$ = ',num2str(RMSE_max),' m/s')};
annotation('textbox','String',str,'FitBoxToText','on','Interpreter','latex','FontSize',20);
xlabel(ax16,'time [s]','Interpreter','latex');
ylabel(ax16,'forward velocity error [m/s]','Interpreter','latex');
ylim([0 0.5]); xlim([0 max([radar_time_second(end) twist_time_second(end)])]);
h1 = legend('mean estimate error','max estimate error');
set(h1,'Interpreter','latex','Location','best');


%% SECONDARY ANALYSIS

% Point Filtering Ideas
%   1. Remove targets near sensor origin
%   2. Remove targets outside of relaible FOV
%   3. Remove targets below a certain intensity threshold

% Averaging
%   1. Weighted averaging - assign weights by intensity

function [ ] = addViconData( h, ax, time, data )

    h(end+1) = plot(ax,time,data,'color',[0.9290, 0.6940, 0.1250],'LineWidth',1);
    h1 = legend([h(1),h(end)],{'Radar Doppler','$v_x$ - Vicon'});
    set(h1,'Interpreter','latex','Location','best');
    
    % move Vicon Data to back of plot
    uistack(h(end),'bottom')

end
