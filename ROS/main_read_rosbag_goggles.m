% program: main_read_ros_bag_goggles.m
% updated: 16-Oct-2019

% This routine reads radar and lidar data from a ROS Bag and 
% saves messages to a mat-file.

% No coordinate transformations are performed. Thus, all 
% variables are in ROS measurement reference frame. 
% x = in front of sensor
% y = + to the left of sensor, - to the right of sensor

clc;
clear;

%% Define the location of input and output files

% ===========================================
% User needs to make changes in this section.
% ===========================================

% Input file information
input_directory   = '/home/carl/Data/icra_2020/radar-quad/uas_flight_space/2019-09-11/goggles/bagfiles/';
filename_root     = 'strafing';
filename_suffix   = '_goggles';
input_suffix      = '.bag';

% Output file information
output_directory  = '/home/carl/Data/icra_2020/radar-quad/uas_flight_space/2019-09-11/goggles/mat_files/';
output_suffix     = '.mat';

% topic information
goggles_topic       = '/mmWaveDataHdl/velocity';

% ============================================
% no User changes are needed below this point.
% ============================================

%% Verify that the input file exists

% define the input and output filenames
input_filename  = [input_directory,filename_root,filename_suffix,input_suffix];
output_filename = [output_directory,filename_root,filename_suffix,output_suffix];

% Does the input file exist?
good_filename = exist(input_filename,'file');

if(good_filename ~= 2)
   disp(['Stop: This filename does not exist: ',input_filename]);
   pause
else
   disp(' ')
   disp(['Processing bag filename: ',input_filename]);
   disp(' ')
end % end if(good_filename ~= 2)

%% Load the ROS bag

bag = rosbag(input_filename);
% The properties are:
%           FilePath: 'C:\Projects_2018_10\DARPA\Fleming_Vicon_2018_1107\bag_files\1642_static_RL_cfar_5120.bag'
%           StartTime: 1.5416e+09
%             EndTime: 1.5416e+09
%         NumMessages: 4086
%     AvailableTopics: [3x3 table]
%     AvailableFrames: {6x1 cell}
%         MessageList: [4086x4 table]

%% Display the AvailableTopics

bag.AvailableTopics;
% The AvailableTopics are:
%                            NumMessages          MessageType                                                                                                                                                                                     MessageDefinition                                                                                                                                                                            
%                            ___________    _______________________
%                            ________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
%
%    /mmWaveDataHdl/RScan         98        sensor_msgs/PointCloud2    '  uint32 Seq?  Time Stamp?  char FrameId?uint32 Height?uint32 Width?sensor_msgs/PointField[] Fields?  uint8 INT8?  uint8 UINT8?  uint8 INT16?  uint8 UINT16?  uint8 INT32?  uint8 UINT32?  uint8 FLOAT32?  uint8 FLOAT64?  char Name?  uint32 Offset?  uint8 Datatype?  uint32 Count?logical IsBigendian?uint32 PointStep?uint32 RowStep?uint8[] Data?logical IsDense?'
%    /scan                        98        sensor_msgs/LaserScan      '  uint32 Seq?  Time Stamp?  char FrameId?single AngleMin?single AngleMax?single AngleIncrement?single TimeIncrement?single ScanTime?single RangeMin?single RangeMax?single[] Ranges?single[] Intensities?'                                                                                                                                                             
%    /tf                        3890        tf2_msgs/TFMessage         '  std_msgs/Header Header?    uint32 Seq?    Time Stamp?    char FrameId?  char ChildFrameId?  geometry_msgs/Transform Transform?    geometry_msgs/Vector3 Translation?      double X?      double Y?      double Z?    geometry_msgs/Quaternion Rotation?      double X?      double Y?      double Z?      double W?'                                                 

%% Display the MessageList

bag.MessageList;
% ans =
% 
%   4086x4 table
% 
%        Time              Topic                  MessageType          FileOffset
%     __________    ____________________    _______________________    __________
% 
%     1.5416e+09    /tf                     tf2_msgs/TFMessage             6480  
%     1.5416e+09    /tf                     tf2_msgs/TFMessage             8944  
%     1.5416e+09    /tf                     tf2_msgs/TFMessage            11401  
%     ...
%     1.5416e+09    /mmWaveDataHdl/RScan    sensor_msgs/PointCloud2       14756 
%     ...
%     1.5416e+09    /scan                   sensor_msgs/LaserScan         23225    
%     ...

%% Get the Goggles messages
% =======================

goggles_bag = select(bag, 'Topic', goggles_topic);
goggles_messages = readMessages(goggles_bag);

%% Display the contents of one radar message cell

%goggles_messages{1}
%   ROS TwistWithCovarianceStamped message with properties:
% 
%     MessageType: 'geometry_msgs/TwistWithCovarianceStamped'
%          Header: [1×1 Header]
%           Twist: [1×1 TwistWithCovariance]

%goggles_messages{1}.Twist
%   ROS TwistWithCovariance message with properties:
% 
%     MessageType: 'geometry_msgs/TwistWithCovariance'
%           Twist: [1×1 Twist]
%      Covariance: [36×1 double]

%goggles_messages{1}.Twist.Twist
%   ROS Twist message with properties:
% 
%     MessageType: 'geometry_msgs/Twist'
%          Linear: [1×1 Vector3]
%         Angular: [1×1 Vector3]


%% Get all radar measurements, for all variables, and for all messages

% pre-define the output variables
[m,~] = size(goggles_messages);

goggles_time_stamp = ones(m,1);

goggles_velocity_linear_x = ones(m,1) .* NaN;
goggles_velocity_linear_y = ones(m,1) .* NaN;
goggles_velocity_linear_z = ones(m,1) .* NaN;

% process each meassage
for r = 1:m
    
   goggles_time_stamp(r) = goggles_messages{r}.Header.Stamp.Sec + ...
       (1e-9)*goggles_messages{r}.Header.Stamp.Nsec;
    
   goggles_velocity_linear_x(r) = goggles_messages{r}.Twist.Twist.Linear.X;
   goggles_velocity_linear_y(r) = goggles_messages{r}.Twist.Twist.Linear.Y;
   goggles_velocity_linear_z(r) = goggles_messages{r}.Twist.Twist.Linear.Z;
   
end % end for r loop

goggles_time_second = goggles_time_stamp - goggles_time_stamp(1);

%% Save Radar messages to a mat-file

save(output_filename, ...
     'goggles_time_stamp','goggles_time_second', ...
     'goggles_velocity_linear_x','goggles_velocity_linear_y','goggles_velocity_linear_z');


