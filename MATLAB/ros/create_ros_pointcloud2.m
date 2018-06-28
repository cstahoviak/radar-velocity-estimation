clear;
clc;
close all;

% fID = fopen('ros_radar_data.txt','r');
% data = fscanf(fID,'%d');
% data2 = textscan(fID,'%d');

% read in ROS PointCloud2 Data - this data taken from a screenshot of the
% rostopic /?? and the 'data' variable saved to a CSV file.
data = csvread('ros_radar_data.csv');

% define ROS PointCloud2 object
ptcloud2 = rosmessage('sensor_msgs/PointCloud2')

% add header info
ptcloud2.Header.Seq = 823;
ptcloud2.Header.Stamp.Sec = 0;
ptcloud2.Header.Stamp.Nsec = 0;
ptcloud2.Header.FrameId = 'base_radar_link';

% add additional info
ptcloud2.Height = 1;
ptcloud2.Width = 36;
ptcloud2.IsBigendian = false;
ptcloud2.PointStep = 32;
ptcloud2.RowStep = 1152;
ptcloud2.IsDense = true;

% add 1D data
ptcloud2.Data = data;

% create and add Point Fields
XField = rosmessage('sensor_msgs/PointField');
YField = rosmessage('sensor_msgs/PointField');
ZField = rosmessage('sensor_msgs/PointField');
IntensityField = rosmessage('sensor_msgs/PointField');

XField.Name = 'x';
XField.Offset = 0;
XField.Datatype = 7;
XField.Count = 1;

YField.Name = 'y';
YField.Offset = 4;
YField.Datatype = 7;
YField.Count = 1;

ZField.Name = 'z';
ZField.Offset = 8;
ZField.Datatype = 7;
ZField.Count = 1;

IntensityField.Name = 'intensity';
IntensityField.Offset = 16;
IntensityField.Datatype = 7;
IntensityField.Count = 1;

% add PointFields to the PointCloud2 object
ptcloud2.Fields = [XField, YField, ZField, IntensityField]

scatter3(ptcloud2)


