% first time run startup_rvc
close all; clear all; clc;

%% Connect to the Rasp Pi
% Get you Pi's IP (type hostname -I into Pi terminal)
% IP = '172.19.226.67';
% IP = 'AlisterCameronPi';
IP = '172.19.232.138';
% IP = 'ohCam';
pb = PiBot(IP);

%% Beancos
b0 = [545, 545];
b1 = [1510, 545];
b2 = [545, 1510];
b3 = [1510, 1510];

Q = [b0',b1',b2',b3'];

tags = pb.getTags()

P = [tags(1,2), tags(2,2), tags(3,2), tags(4,2);
    tags(1,3), tags(2,3), tags(3,3), tags(4,3)];

H = homography(P, Q);

%homtrans