matlabrc; clc; close all;
addpath(genpath('data'))
addpath(genpath('OLD'))

img = imread('frame_000.png');
% imshow(img)

% Load in the camera model:
load('cam')
load('truth')
r = positions(1,:)';
v = velocities(1,:)';

% Project Earth points:

% Simpler debug, plot center point of image:
imshow(img); hold on
[n,m,~] = size(img);
plot(m/2,n/2,'.r','MarkerSize',20)