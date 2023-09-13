clear; close all; clc; rosshutdown;

rosinit("10.211.55.6");

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
params = lidarParameters('OS1Gen1-32', 1024);

roi = [0, 20,-10, 10, -1, 2];

initPlayer

while true
    receivedPoints = lidarSubscriber.receive(params);

    roiPoints = getPointsInROI(receivedPoints, roi);
    view(roiPyr, roiPoints);

    nonGroundPoints = getNonGroundPoints(roiPoints, 0.06, [0,0,1], 30);
    view(grmPyr, nonGroundPoints);

    denoisedPoints = pcdenoise(nonGroundPoints);
    view(denPyr, denoisedPoints);
end


