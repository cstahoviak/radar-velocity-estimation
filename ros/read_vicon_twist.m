%% Define the location of input and output files

% ===========================================
% User needs to make changes in this section.
% ===========================================

% Input file information
input_directory   = '/home/carl/Data/subT/vicon_velocity_estimate_110818/1642/';
filename_root     = 'linear_best_range_res';
input_suffix      = '.bag';

% Output file information
output_directory  = '/home/carl/Data/subT/vicon_velocity_estimate_110818/1642/mat_files/';
output_suffix     = '.mat';

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

%% Get the Vicon Twist messages
% =======================

twist_bag   = select(bag, 'Topic', '/vrpn_client_node/SubT/twist');

% The properties are:


%% Get the time stamp for all radar messages

time_stamp_table  = twist_bag.MessageList(:,1);
twist_time_stamp  = time_stamp_table{:,1};
twist_time_second = twist_time_stamp - twist_time_stamp(1);

%% Read all messages

twist_messages = readMessages(twist_bag);

% return;


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


%% Display the radar Field Names

%radar_messages{1}.Fields.Name

% ans =   'x' 'y' 'z' 'intensity' 'range' 'doppler'

%% Get all radar measurements, for all variables, and for all messages

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