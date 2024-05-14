% INITIALIZE
clear
clc
close all

Tf = 3*60; % Time of running the model

% State Initialization
x0 = [85.0;       % flying at velocity of 85m/s or 165 knots
    0.0
    0.0
    0.0
    0.0
    0.0
    0.0
    0.1         % approx 5.73 deg climbing
    0.0];
% Control Initialization
u = [0.0;
    -0.1;       % -5.73 deg
    0.0
    0.08;       % recall min 0.5*pi/180 = 0.0087
    0.08];


% % From Numerically Calculated Trim Function
% temp = load("trim_vales.mat");
% x0 = temp.Xstar;
% u = temp.Ustar;



% --------------------Control Limits--------------------------------------

% Alieron Limit
u1min = -25*pi/180;
u1max = 25*pi/180;
% Elevator Limit
u2min = -25*pi/180;
u2max = 10*pi/180;
% Rudder Limit
u3min = -30*pi/180;
u3max = 30*pi/180;
% Engine 1 Throttle Limit
u4min = 0;
u4max = 10*pi/180;
% Engine 2 Throttle Limit
u5min = 0;
u5max = 10*pi/180;