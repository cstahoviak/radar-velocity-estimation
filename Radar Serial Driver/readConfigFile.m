%% Header

clear;
clc;
close all;

%%

text = fileread('1642_2d_sdk_1_2.cfg');
C = strsplit(text,'\n');
out = string(C{1});
out2 = C{1};

