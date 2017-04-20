% first time run startup_rvc
close all; clear all; clc;

%% Connect to the Rasp Pi
% Get you Pi's IP (type hostname -I into Pi terminal)
% IP = '172.19.226.67';
% IP = 'AlisterCameronPi';
IP = '172.19.232.138';
% IP = 'ohCam';
pb = PiBot(IP);

tests = 100;

%P = zeros(2,4);

for i = 1:tests
    tags = pb.getTags();
    p(:,:,i) = [tags(1,2), tags(2,2), tags(3,2), tags(4,2);
    tags(1,3), tags(2,3), tags(3,3), tags(4,3)];
    pause(0.1);
end

Pstd = std(p,0,3)

P = mean(p,3)



