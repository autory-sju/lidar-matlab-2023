clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
detectionSubscriber = rossubscriber("/yolov5/cob_detections");

load("calibration/camera_cali_result_9.16.20.mat")
load("calibration/lcc-2023-09-17-00.mat")

params = lidarParameters('OS1Gen1-32', 1024);
roi = [0, 10,-5, 5, -1, 2];

conPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive(params);
    bboxData = receive(detectionSubscriber);

    roiPoints = getPointsInROI(receivedPoints, roi);
    nonGroundPoints = getNonGroundPoints(roiPoints, 0.06, [0,0,1], 30); % model, inlierIndices, outlierIndices, meanError
    denoisedPoints = pcdenoise(nonGroundPoints);

    bboxesCamera = getBboxesCamera(bboxData);
    [bboxesLidar,~,bboxesUsed] = bboxCameraToLidar(bboxesCamera, denoisedPoints, cameraParams, invert(tform), 'ClusterThreshold',0.2);
    [yBboxes, bBboxes, rBboxes] = classifyBboxLidar(bboxesLidar, bboxData, bboxesUsed);

    view(conPyr, denoisedPoints)
    showShape("cuboid",[yBboxes; bBboxes; rBboxes], ...
        Parent=conPyr.Axes, ...
        Color="green", ...
        Opacity=0.5)
    drawnow

    yPos = yBboxes(:, 1:2);
    rPos = rBboxes(:, 1:2);
    bPos = bBboxes(:, 1:2);
end