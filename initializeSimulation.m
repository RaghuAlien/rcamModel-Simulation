% INITIALIZE
clear
clc
close all
load ("trim_vales.mat")
Tf = 3*60; % Time of running the model

% Trim values Initialization
% State Initialization
x0 = Xstar;
% Control Initialization
u = Ustar;


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

% Latidude & Longitude Position

long0 = deg2rad(-22.56242);     % Longitude for Keflavík 
lat0 = deg2rad(64.00492);       % Latitude for Keflavík 
h0 = 500;

Xgeodetic0 = [lat0;long0;h0];

