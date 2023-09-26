clear; close all; clc; rosshutdown;

rosinit("10.211.55.6");

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");

params = lidarParameters('OS1Gen1-32', 1024);

roi = [0, 20,-10, 10, -1, 2];

roiPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));
proPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive(params);

    roiPoints = getPointsInROI(receivedPoints, roi);

    [~,nonGroundPoints,~] = segmentGroundSMRF(roiPoints, 1, ...
        "MaxWindowRadius",18, ...
        "SlopeThreshold",0.15, ...
        "ElevationThreshold",0.03, ...
        "ElevationScale",1.25);
    % nonGroundPoints = getNonGroundSMRF(roiPoints);

    view(roiPyr, roiPoints);
    view(proPyr, nonGroundPoints);
end