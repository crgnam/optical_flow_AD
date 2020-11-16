clear; matlabrc; clc; close all;
addpath(genpath('src'))
addpath(genpath('lib'))
path = 'data/';
file_list = dir([path,'frame*.csv']);

% Read in truth data:
load([path,'truth'])

% Read in sensor data:
load([path,'cam'])
load([path,'gps'])
load([path,'gyro'])

for ii = 1:length(file_list)
    % Read in the image solution data:
    [roll,pitch,pts1,pts2] = read_image_data([path,file_list(ii).name]);
    
    % Run filter:
    
end