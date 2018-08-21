%% Header

% Before running this script start the following from terminal
% 1. roslaunch ti_mmwave_rospkg rviz_1642_2d.launch
% 2. roslaunch sweep_ros sweep.launch
% 3. run 'rosinit' from the command window

% This will create the ROS topics '/mmWaveDataHdl/Rscan' from the radar and
% '/pc2' from the lidar which we will subscribe to

clear;
clc;
close all;

%% Functionality

% define radar/lidar data figure
fig1 = figure(1);
ax1 = axes('Parent',fig1);
grid on;

rdr_callback_handle = {@rdr_callback,ax1};
ldr_callback_handle = {@ldr_callback,ax1};

% init radar and lidar subscribers
rdr_sub = rossubscriber('/mmWaveDataHdl/RScan','sensor_msgs/PointCloud2',rdr_callback_handle);
ldr_sub = rossubscriber('/pc2','sensor_msgs/PointCloud2',ldr_callback_handle);

% rdr_pc2 = receive(rdr_sub,10);
% ldr_pc2 = receive(ldr_sub,10);
% return;

% while true
%     
% shutdown = onCleanup(@() endROSProcess());
% 
%     
% end

while time < 10
    time = toc;
end

rosshutdown;
return;

function rdr_callback(~,rdr_pc2,ax1)
    rdr_xyz = readXYZ(rdr_pc2);
    
    % note: lidar intensity data not available
    rdr_intensity = readField(rdr_pc2, 'intensity');
    rdr_intensity = (rdr_intensity./100);
    
    rdr_data = [rdr_xyz rdr_intensity];
    plot_pc2(rdr_data,'rdr',ax1);
end

function ldr_callback(~,ldr_pc2,ax1)
    ldr_xyz = readXYZ(ldr_pc2);
    
    plot_pc2(ldr_xyz,'ldr',ax1)
end

function plot_pc2(data,type,ax)
    % persist both rdr_data and ldr_data from one function call to the next
    persistent rdr_data;
    persistent ldr_data;
    
    if strcmp(type,'rdr')
        rdr_data = data;
        
    elseif strcmp(type,'ldr')
        ldr_data = data;
         
    else
        fprintf('Error in plot_pc2(): data type improperly specified');
        rosshutdown;
        return;
    end
    
      % From ti_mmwave_rospkg DataHandlerClass.cpp
%     temp[0] = (float) mmwData.objOut.x;
%     temp[1] = (float) mmwData.objOut.y;
%     temp[2] = (float) mmwData.objOut.z;
%     
%     // Map mmWave sensor coordinates to ROS coordinate system
%     RScan->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
%     RScan->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
%     RScan->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
    
    % update data for new received message and undo ROS xy re-mapping as
    % described in the comments above (NOT COMPLETE)
    rdr_x = rdr_data(:,2)*(-1);
    rdr_y = rdr_data(:,1);
    rdr_intensity = rdr_data(:,4);
        
    ldr_x = ldr_data(:,2)*(-1);
    ldr_y = ldr_data(:,1);
    
    % plot radar data
    for(i=1:length(rdr_x))
        scatter(ax,rdr_x(i),rdr_y(i),10,'MarkerFaceColor','b', ...
            'MarkerEdgeColor','b','MarkerFaceAlpha',rdr_intensity(i), ...
            'MarkerEdgeAlpha',0)
        ylim(ax,[-5,10]); xlim(ax,[-10,10]);
        grid on;
        hold on;
    end
    
    % plot lidar data
    for(i=1:length(ldr_x))
        scatter(ax,ldr_x(i),ldr_y(i),10,'r','filled')
        ylim(ax,[-5,10]); xlim(ax,[-10,10]);
        grid on;
        hold on;
    end
    hold off;
end

% haven't gotten onCleanUp function to work yet
function endROSProcess()
    rosshutdown;
end