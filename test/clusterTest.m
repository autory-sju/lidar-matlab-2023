clear; close all; clc; rosshutdown;

rosinit("10.211.55.7");

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");

% params = lidarParameters('OS1Gen1-32', 1024);

roi = [0, 20, -10, 10, -1, 2];

% player = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive();

    roiPoints = getPointsInROI(receivedPoints, roi);

    nonGroundPoints = getNonGroundSMRF(roiPoints);

    [labels,numClusters] = pcsegdist(nonGroundPoints,0.3);

    pcshow(nonGroundPoints.Location,labels)
    colormap(hsv(numClusters))
end