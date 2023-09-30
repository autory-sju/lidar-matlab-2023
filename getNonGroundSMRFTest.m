clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");

params = lidarParameters('OS1Gen1-32', 1024);

roi = [0, 20,-10, 10, -1, 2];

roiPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));
proPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive();

    roiPoints = getPointsInROI(receivedPoints, roi);

    nonGroundPoints = getNonGroundSMRF(roiPoints);

    view(roiPyr, receivedPoints);
    view(proPyr, nonGroundPoints);
end