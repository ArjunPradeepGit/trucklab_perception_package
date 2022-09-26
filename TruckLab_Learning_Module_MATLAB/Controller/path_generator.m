clc
clear
close all

% RUN THIS SCRIPT BEFORE THE SIMULINK MODEL
%   --> to generate the path in the workspace

% This script is aimed to control the tractor to run in a straight line.
% Different paths could be generated and provided as input as well, by
% making changes to the path variables.

% User Choice for the path-generation
x1 = 0.0;
y1 = 0.0;
x2 = 5.0;
y2 = 5.0;
total_num_waypoints = 1000;
vel_uniform = 0.1; % m/s
steering_rate_limit = 100; % deg/sec  (steering command limiter)
freq = 100; % hz  (simulink freq)

% Path generation
% Note that the path variables have to be inputed with time
slope = (y2-y1)/(x2-x1);
travel_dist = sqrt((x2-x1).^2 + (y2-y1).^2);
x = linspace(x1, x2, total_num_waypoints)';
y = linspace(y1, y2, total_num_waypoints)';
theta = linspace(atan2(slope,1), atan2(slope,1), total_num_waypoints)'.*180/pi;
vel = linspace(vel_uniform, vel_uniform, total_num_waypoints)';
time = linspace(0.0, travel_dist/vel_uniform, total_num_waypoints)';

ref_Kf = 4;
ref_Kr = 0.0000001;

ref_pose = [time, x, y, theta];
ref_vel = [time, vel];