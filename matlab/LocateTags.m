%% clear work space
% run startup_rvc on first start
close all
clear all
clc

%chage to true to test without camera
debug = true;

tagCent = 0.03; %half the width of the tag

% [bottom left; top right; robot]
% elements of markerTags and beaconTags
% [tag id, u, v, u corner, v corner, x(m), y(m), theta(rad)]
markerIDs = [0;1;4];
beaconIDs = [5;6;7];

%% 
IP = '192.168.1.139';
pb = PiBot(IP); 


%%
tagsOK = false;
while ~ tagsOK
    if debug
       load('C:\Users\Troy\Documents\University\ENB439_Tutoring\EGB439\matlab\DataSet.mat')
    else
        tempTags = pb.getTags();
    end
    % add check that bottom left and top right are found
    markerTags = tags(ismember( tags(:,1), markerIDs ),:);
    beaconTags = tags(ismember( tags(:,1), beaconIDs ),:);
    
    if size(markerTags,1) == 3
        tagsOK = true;
    else
        pause(0.1)
        imageStat = sprintf('Retacking Image');
        disp(imageStat)
    end
end


xScale = (2.0-(2*tagCent))/(markerTags(2,2) - markerTags(1,2));
yScale = (2.0-(2*tagCent))/(markerTags(1,3) - markerTags(2,3));

% scale x dimention to meters
markerTags(:,6) = ((markerTags(:,2) - markerTags(1,2)) * xScale) + tagCent;
beaconTags(:,6) = ((beaconTags(:,2) - markerTags(1,2)) * xScale) + tagCent;


% scale y dimention to meters
markerTags(:,7) = ((markerTags(1,3) - markerTags(:,3)) * yScale) + tagCent;
beaconTags(:,7) = ((markerTags(1,3) - beaconTags(:,3)) * yScale) + tagCent;

RCx = ((markerTags(3,4) - markerTags(1,2)) * xScale) + tagCent;
RCy = ((markerTags(1,5) - markerTags(3,3)) * yScale) + tagCent;

markerTags(3,8) = atan2(RCy-markerTags(3,7),RCx-markerTags(3,6)) - pi/4;
markerTags(3,8) = atan2(RCy-markerTags(3,7),RCx-markerTags(3,6)) - pi/4;

Tr = (se2(markerTags(3,6),markerTags(3,7), markerTags(3,8)));

figure(1)
hold on
plot(markerTags(1:2,6),markerTags(1:2,7),'bx');
plot(markerTags(3,6),markerTags(3,7),'ks');
plot(beaconTags(:,6),beaconTags(:,7),'ro');
hold off

nobeaconTags = size(beaconTags,1);
beaconTagsR = zeros(nobeaconTags,2);

relRobot = sprintf('Beacons relative to the robot');
disp(relRobot)
for i = 1:nobeaconTags
    beaconTagsR(i,1:2) = homtrans( inv(Tr), [beaconTags(i,6); beaconTags(i,7)]);
    relRobot = sprintf('Beacon %d: (%f,%f)', beaconTags(i,1), beaconTagsR(i,1), beaconTagsR(i,2));
    disp(relRobot)
end

figure(2)
hold on
plot(beaconTagsR(:,1),beaconTagsR(:,2),'ro');
plot(0,0,'ks');
hold off

