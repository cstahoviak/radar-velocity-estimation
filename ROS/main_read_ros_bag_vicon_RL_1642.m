% program: main_read_ros_bag_vicon_RL_1642.m
% updated: 8-Nov-2018

% This routine reads radar and lidar data from a ROS Bag and 
% saves messages to a mat-file.

% No coordinate transformations are performed. Thus, all 
% variables are in ROS measurement reference frame. 
% x = in front of sensor
% y = + to the left of sensor, - to the right of sensor

%% Define the location of input and output files

% ===========================================
% User needs to make changes in this section.
% ===========================================

% Input file information
input_directory   = '/home/carl/Data/subT/Fleming/fleming_radar_2019-05-17/cfar-800/';
filename_root     = 'cfar-800_10Hz_run2';
input_suffix      = '.bag';

% Output file information
output_directory  = '/home/carl/Data/subT/Fleming/fleming_radar_2019-05-17/mat_files/';
output_suffix     = '.mat';

% topic information
radar_topic       = '/mmWaveDataHdl/RScan';
% radar_fwd_topic   = '/radar_fwd/mmWaveDataHdl/RScan';
% radar_lat_topic   = '/radar_lat/mmWaveDataHdl/RScan';
vrpn_twist_topic  = '/vrpn_client_node/A01Radar/twist';
vrpn_pose_topic   = '/vrpn_client_node/A01Radar/pose';
odom_topic        = '/odom';
tf_topic          = '/tf';

% ============================================
% no User changes are needed below this point.
% ============================================

%% Verify that the input file exists

% define the input and output filenames
input_filename    = [input_directory,filename_root,input_suffix];
output_filename    = [output_directory,filename_root,output_suffix];

% Does the input file exist?
good_filename     = exist(input_filename,'file');

if(good_filename ~= 2)
   disp(['Stop: This filename does not exist: ',input_filename]);
   pause
else
   disp(' ')
   disp(['Processing bag filename: ',input_filename]);
   disp(' ')
end % end if(good_filename ~= 2)

%% Load the ROS bag

bag   = rosbag(input_filename);
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

%% Get the radar messages
% =======================

radar_bag   = select(bag, 'Topic', radar_topic);
% The properties are:
%            FilePath: 'C:\Projects_2018_10\DARPA\Fleming_Vicon_2018_1107\bag_files\1642_static_RL_cfar_5120.bag'
%           StartTime: 1.5416e+09
%             EndTime: 1.5416e+09
%         NumMessages: 98
%     AvailableTopics: [1x3 table]
%     AvailableFrames: {6x1 cell}
%         MessageList: [98x4 table]

%% Get the time stamp for all radar messages

time_stamp_table  = radar_bag.MessageList(:,1);
radar_time_stamp  = time_stamp_table{:,1};
radar_time_second = radar_time_stamp - radar_time_stamp(1);

%% Read all messages

radar_messages = readMessages(radar_bag);
%   98�1 cell array
% 
%     {1�1 PointCloud2}
%     {1�1 PointCloud2}
%     {1�1 PointCloud2}
%     ...

%% Display the contents of one radar message cell

%radar_messages{1}
%   ROS PointCloud2 message with properties:
% 
%     PreserveStructureOnRead: 0
%                 MessageType: 'sensor_msgs/PointCloud2'
%                      Header: [1x1 Header]
%                      Height: 1
%                       Width: 75
%                 IsBigendian: 0
%                   PointStep: 32
%                     RowStep: 2400
%                     IsDense: 1
%                      Fields: [6x1 PointField]
%                        Data: [2400x1 uint8]

%% Display the radar Field Names

%radar_messages{1}.Fields.Name

% ans =   'x' 'y' 'z' 'intensity' 'range' 'doppler'

%% Get all radar measurements, for all variables, and for all messages

% pre-define the output variables
[m,~]    = size(radar_messages);

max_num_targets   = 350;

radar_x           = ones(m,max_num_targets) .* NaN;
radar_y           = ones(m,max_num_targets) .* NaN;
radar_z           = ones(m,max_num_targets) .* NaN;
radar_intensity   = ones(m,max_num_targets) .* NaN;
radar_range       = ones(m,max_num_targets) .* NaN;
radar_doppler     = ones(m,max_num_targets) .* NaN;

% process each meassage

for r = 1:m
   
   temp_x           = readField(radar_messages{r}, 'x');
   temp_y           = readField(radar_messages{r}, 'y');
   temp_z           = readField(radar_messages{r}, 'z');
   temp_intensity   = readField(radar_messages{r}, 'intensity');
   temp_range       = readField(radar_messages{r}, 'range');
   temp_doppler     = readField(radar_messages{r}, 'doppler');
   
   % put these values into the master matrices
   n = length(temp_x);

   radar_x(r,1:n)          = temp_x;
   radar_y(r,1:n)          = temp_y;
   radar_z(r,1:n)          = temp_z;
   radar_intensity(r,1:n)  = temp_intensity;
   radar_range(r,1:n)      = temp_range;
   radar_doppler(r,1:n)    = temp_doppler;
   
end % end for r loop

% %% Get the radar_fwd messages
% % =======================
% 
% radar_fwd_bag   = select(bag, 'Topic', radar_fwd_topic);
% % The properties are:
% %            FilePath: 'C:\Projects_2018_10\DARPA\Fleming_Vicon_2018_1107\bag_files\1642_static_RL_cfar_5120.bag'
% %           StartTime: 1.5416e+09
% %             EndTime: 1.5416e+09
% %         NumMessages: 98
% %     AvailableTopics: [1x3 table]
% %     AvailableFrames: {6x1 cell}
% %         MessageList: [98x4 table]
% 
% %% Get the time stamp for all radar messages
% 
% time_stamp_table  = radar_fwd_bag.MessageList(:,1);
% radar_fwd_time_stamp  = time_stamp_table{:,1};
% radar_fwd_time_second = radar_fwd_time_stamp - radar_fwd_time_stamp(1);
% 
% %% Read all messages
% 
% radar_fwd_messages = readMessages(radar_fwd_bag);
% %   98�1 cell array
% % 
% %     {1�1 PointCloud2}
% %     {1�1 PointCloud2}
% %     {1�1 PointCloud2}
% %     ...
% 
% %% Display the contents of one radar message cell
% 
% %radar_messages{1}
% %   ROS PointCloud2 message with properties:
% % 
% %     PreserveStructureOnRead: 0
% %                 MessageType: 'sensor_msgs/PointCloud2'
% %                      Header: [1x1 Header]
% %                      Height: 1
% %                       Width: 75
% %                 IsBigendian: 0
% %                   PointStep: 32
% %                     RowStep: 2400
% %                     IsDense: 1
% %                      Fields: [6x1 PointField]
% %                        Data: [2400x1 uint8]
% 
% %% Display the radar Field Names
% 
% %radar_messages{1}.Fields.Name
% 
% % ans =   'x' 'y' 'z' 'intensity' 'range' 'doppler'
% 
% %% Get all radar measurements, for all variables, and for all messages
% 
% % pre-define the output variables
% [m,~]    = size(radar_fwd_messages);
% 
% max_num_targets   = 100;
% 
% radar_fwd_x           = ones(m,max_num_targets) .* NaN;
% radar_fwd_y           = ones(m,max_num_targets) .* NaN;
% radar_fwd_z           = ones(m,max_num_targets) .* NaN;
% radar_fwd_intensity   = ones(m,max_num_targets) .* NaN;
% radar_fwd_range       = ones(m,max_num_targets) .* NaN;
% radar_fwd_doppler     = ones(m,max_num_targets) .* NaN;
% 
% % process each meassage
% 
% for r = 1:m
%    
%    temp_x           = readField(radar_fwd_messages{r}, 'x');
%    temp_y           = readField(radar_fwd_messages{r}, 'y');
%    temp_z           = readField(radar_fwd_messages{r}, 'z');
%    temp_intensity   = readField(radar_fwd_messages{r}, 'intensity');
%    temp_range       = readField(radar_fwd_messages{r}, 'range');
%    temp_doppler     = readField(radar_fwd_messages{r}, 'doppler');
%    
%    % put these values into the master matrices
%    n = length(temp_x);
% 
%    radar_fwd_x(r,1:n)          = temp_x;
%    radar_fwd_y(r,1:n)          = temp_y;
%    radar_fwd_z(r,1:n)          = temp_z;
%    radar_fwd_intensity(r,1:n)  = temp_intensity;
%    radar_fwd_range(r,1:n)      = temp_range;
%    radar_fwd_doppler(r,1:n)    = temp_doppler;
%    
% end % end for r loop
% 
% %% Get the radar_lat messages
% % =======================
% 
% radar_lat_bag   = select(bag, 'Topic', radar_lat_topic);
% % The properties are:
% %            FilePath: 'C:\Projects_2018_10\DARPA\Fleming_Vicon_2018_1107\bag_files\1642_static_RL_cfar_5120.bag'
% %           StartTime: 1.5416e+09
% %             EndTime: 1.5416e+09
% %         NumMessages: 98
% %     AvailableTopics: [1x3 table]
% %     AvailableFrames: {6x1 cell}
% %         MessageList: [98x4 table]
% 
% %% Get the time stamp for all radar messages
% 
% time_stamp_table  = radar_lat_bag.MessageList(:,1);
% radar_lat_time_stamp  = time_stamp_table{:,1};
% radar_lat_time_second = radar_lat_time_stamp - radar_lat_time_stamp(1);
% 
% %% Read all messages
% 
% radar_lat_messages = readMessages(radar_lat_bag);
% %   98�1 cell array
% % 
% %     {1�1 PointCloud2}
% %     {1�1 PointCloud2}
% %     {1�1 PointCloud2}
% %     ...
% 
% %% Display the contents of one radar message cell
% 
% %radar_messages{1}
% %   ROS PointCloud2 message with properties:
% % 
% %     PreserveStructureOnRead: 0
% %                 MessageType: 'sensor_msgs/PointCloud2'
% %                      Header: [1x1 Header]
% %                      Height: 1
% %                       Width: 75
% %                 IsBigendian: 0
% %                   PointStep: 32
% %                     RowStep: 2400
% %                     IsDense: 1
% %                      Fields: [6x1 PointField]
% %                        Data: [2400x1 uint8]
% 
% %% Display the radar Field Names
% 
% %radar_messages{1}.Fields.Name
% 
% % ans =   'x' 'y' 'z' 'intensity' 'range' 'doppler'
% 
% %% Get all radar measurements, for all variables, and for all messages
% 
% % pre-define the output variables
% [m,~]    = size(radar_lat_messages);
% 
% max_num_targets   = 100;
% 
% radar_lat_x           = ones(m,max_num_targets) .* NaN;
% radar_lat_y           = ones(m,max_num_targets) .* NaN;
% radar_lat_z           = ones(m,max_num_targets) .* NaN;
% radar_lat_intensity   = ones(m,max_num_targets) .* NaN;
% radar_lat_range       = ones(m,max_num_targets) .* NaN;
% radar_lat_doppler     = ones(m,max_num_targets) .* NaN;
% 
% % process each meassage
% 
% for r = 1:m
%    
%    temp_x           = readField(radar_lat_messages{r}, 'x');
%    temp_y           = readField(radar_lat_messages{r}, 'y');
%    temp_z           = readField(radar_lat_messages{r}, 'z');
%    temp_intensity   = readField(radar_lat_messages{r}, 'intensity');
%    temp_range       = readField(radar_lat_messages{r}, 'range');
%    temp_doppler     = readField(radar_lat_messages{r}, 'doppler');
%    
%    % put these values into the master matrices
%    n = length(temp_x);
% 
%    radar_lat_x(r,1:n)          = temp_x;
%    radar_lat_y(r,1:n)          = temp_y;
%    radar_lat_z(r,1:n)          = temp_z;
%    radar_lat_intensity(r,1:n)  = temp_intensity;
%    radar_lat_range(r,1:n)      = temp_range;
%    radar_lat_doppler(r,1:n)    = temp_doppler;
%    
% end % end for r loop

%% Get the Vicon Twist messages
% =======================

twist_bag   = select(bag, 'Topic', vrpn_twist_topic);

% The properties are:


%% Get the time stamp for all Twist messages

time_stamp_table  = twist_bag.MessageList(:,1);
twist_time_stamp  = time_stamp_table{:,1};
twist_time_second = twist_time_stamp - twist_time_stamp(1);

%% Read all Twist messages

twist_messages = readMessages(twist_bag);


%% Display the contents of one twist message cell

% twist_messages{1}
%   ROS TwistStamped message with properties:
% 
%     MessageType: 'geometry_msgs/TwistStamped'
%          Header: [1×1 Header]
%           Twist: [1×1 Twist]

% twist_messages{1}.Twist
%   ROS Twist message with properties:
% 
%     MessageType: 'geometry_msgs/Twist'
%          Linear: [1×1 Vector3]
%         Angular: [1×1 Vector3]

% twist_messages{1}.Twist.Linear
%   ROS Vector3 message with properties:
% 
%     MessageType: 'geometry_msgs/Vector3'
%               X: -0.0172
%               Y: -0.0118
%               Z: 0.0264


%% Get all Twist measurements, for all variables, and for all messages

% pre-define the output variables
[m,~]    = size(twist_messages);

twist_linear_x           = ones(m,1) .* NaN;
twist_linear_y           = ones(m,1) .* NaN;
twist_linear_z           = ones(m,1) .* NaN;
twist_angular_x          = ones(m,1) .* NaN;
twist_angular_y          = ones(m,1) .* NaN;
twist_angular_z          = ones(m,1) .* NaN;

% process each meassage
for r = 1:m
    
   twist_linear_x(r) = twist_messages{r}.Twist.Linear.X;
   twist_linear_y(r) = twist_messages{r}.Twist.Linear.Y;
   twist_linear_z(r) = twist_messages{r}.Twist.Linear.Z;
   
   twist_angular_x(r) = twist_messages{r}.Twist.Angular.X;
   twist_angular_y(r) = twist_messages{r}.Twist.Angular.Y;
   twist_angular_z(r) = twist_messages{r}.Twist.Angular.Z;
   
end % end for r loop

%% Get the Vicon Pose messages
% =======================

pose_bag   = select(bag, 'Topic', vrpn_pose_topic);

% The properties are:


%% Get the time stamp for all Pose messages

time_stamp_table  = pose_bag.MessageList(:,1);
pose_time_stamp  = time_stamp_table{:,1};
pose_time_second = pose_time_stamp - pose_time_stamp(1);

%% Read all Pose messages

pose_messages = readMessages(pose_bag);

% return;


%% Display the contents of one pose message cell

% pose_messages{1}
%   ROS PoseStamped message with properties:
% 
%     MessageType: 'geometry_msgs/PoseStamped'
%          Header: [1×1 Header]
%            Pose: [1×1 Pose]

% pose_messages{1}.Pose
%   ROS Pose message with properties:
% 
%     MessageType: 'geometry_msgs/Pose'
%        Position: [1×1 Point]
%     Orientation: [1×1 Quaternion]

% pose_messages{1}.Pose.Position
%   ROS Point message with properties:
% 
%    MessageType: 'geometry_msgs/Point'
%               X: -1.9967
%               Y: 0.3000
%               Z: 0.1421

% pose_messages{1}.Pose.Orientation
%   ROS Quaternion message with properties:
% 
%    MessageType: 'geometry_msgs/Quaternion'
%               X: 0.0026
%               Y: -0.0098
%               Z: -0.0153
%               W: 0.9998


%% Get all Pose measurements, for all variables, and for all messages

% pre-define the output variables
[m,~]    = size(pose_messages);

pose_position_x           = ones(m,1) .* NaN;
pose_position_y           = ones(m,1) .* NaN;
pose_position_z           = ones(m,1) .* NaN;
pose_orientation_x        = ones(m,1) .* NaN;
pose_orientation_y        = ones(m,1) .* NaN;
pose_orientation_z        = ones(m,1) .* NaN;
pose_orientation_w        = ones(m,1) .* NaN;

% process each meassage
for r = 1:m
   pose_position_x(r) = pose_messages{r}.Pose.Position.X;
   pose_position_y(r) = pose_messages{r}.Pose.Position.Y;
   pose_position_z(r) = pose_messages{r}.Pose.Position.Z;
   
   pose_orientation_x(r) = pose_messages{r}.Pose.Orientation.X;
   pose_orientation_y(r) = pose_messages{r}.Pose.Orientation.Y;
   pose_orientation_z(r) = pose_messages{r}.Pose.Orientation.Z;
   pose_orientation_w(r) = pose_messages{r}.Pose.Orientation.W;
   
end % end for r loop


%% Save Radar messages to a mat-file

% save(output_filename,'radar_fwd_time_stamp','radar_fwd_time_second', ...
%   'radar_fwd_x','radar_fwd_y','radar_fwd_z','radar_fwd_range','radar_fwd_intensity','radar_fwd_doppler', ...
%   'radar_lat_time_stamp','radar_lat_time_second', ...
%   'radar_lat_x','radar_lat_y','radar_lat_z','radar_lat_range','radar_lat_intensity','radar_lat_doppler', ...
%   'twist_time_stamp','twist_time_second', ...
%   'twist_linear_x','twist_linear_y','twist_linear_z','twist_angular_x','twist_angular_y','twist_angular_z', ...
%   'pose_time_stamp','pose_time_second', ...
%   'pose_position_x', 'pose_position_y', 'pose_position_z', 'pose_orientation_x', 'pose_orientation_y', 'pose_orientation_z', 'pose_orientation_w');

save(output_filename, ...
     'radar_time_stamp','radar_time_second','radar_x','radar_y','radar_z', ...
     'radar_range','radar_intensity','radar_doppler', ...
     'twist_time_stamp','twist_time_second', ...
     'twist_linear_x','twist_linear_y','twist_linear_z','twist_angular_x','twist_angular_y','twist_angular_z', ...
     'pose_time_stamp','pose_time_second', ...
     'pose_position_x', 'pose_position_y', 'pose_position_z', ...
     'pose_orientation_x', 'pose_orientation_y', 'pose_orientation_z', 'pose_orientation_w');

return;

%% Get the Lidar messages
% =======================

lidar_bag   = select(bag, 'Topic', '/scan');
% The properties are:
%            FilePath: 'C:\Projects_2018_10\DARPA\Fleming_Vicon_2018_1107\bag_files\1642_static_RL_cfar_5120.bag'
%           StartTime: 1.5416e+09
%             EndTime: 1.5416e+09
%         NumMessages: 98
%     AvailableTopics: [1�3 table]
%     AvailableFrames: {6�1 cell}
%         MessageList: [98�4 table]

%% Get the time stamp for all lidar messages

time_stamp_table  = lidar_bag.MessageList(:,1);
lidar_time_stamp  = time_stamp_table{:,1};
lidar_time_second = lidar_time_stamp - lidar_time_stamp(1);

%% Read all lidar messages

lidar_messages = readMessages(lidar_bag);
% 98�1 cell array
% 
%  {1�1 LaserScan}
%  {1�1 LaserScan}
%  {1�1 LaserScan}
%     ...

%% Display the contents of one lidar message cell

%lidar_messages{1}
% ROS LaserScan message with properties:
%
%        MessageType: 'sensor_msgs/LaserScan'
%             Header: [1�1 Header]
%           AngleMin: -1.5708
%           AngleMax: 1.5708
%     AngleIncrement: 0.0061
%      TimeIncrement: 9.7656e-05
%           ScanTime: 0.1000
%           RangeMin: 0.0200
%           RangeMax: 5.6000
%             Ranges: [513�1 single]
%        Intensities: [0�1 single]

%% Convert (range, angle) to (x,y) using readCartesian(lidar_messages{1})

% Use this command to get (x,y) for one message:
% xy_matrix = readCartesian(lidar_messages{1});

%% Get all (x,y) values for all lidar messages

% pre-define the output variables
[m,~]    = size(lidar_messages);

max_num_targets   = 250;

lidar_x           = ones(m,max_num_targets) .* NaN;
lidar_y           = ones(m,max_num_targets) .* NaN;

% process each meassage

for r = 1:m
   
   xy_matrix        = readCartesian(lidar_messages{r});
   
   % put these values into the master matrices
   [mm,~]     = size(xy_matrix);

   if(mm > 0)
      lidar_x(r,1:mm)          = xy_matrix(:,1)';
      lidar_y(r,1:mm)          = xy_matrix(:,2)';
   end % end if(mm > 0)   
   
end % end for r loop

%% Save the messages to a mat-file

save(output_filename,'radar_time_stamp','radar_time_second',...
  'radar_x','radar_y','radar_z','radar_range','radar_intensity',...
  'radar_doppler','lidar_time_stamp','lidar_time_second',...
  'lidar_x','lidar_y')
                  