clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
l_detectionSubscriber = rossubscriber("/yolov5/cob_detections"); % need topic
r_detectionSubscriber = rossubscriber("/yolov5/cob_detections"); % need topic

load("calibration/camera_calibration_left.mat")
load("calibration/camera_calibration_right.mat")
load("calibration/lidar_camera_calibration_left.mat")
load("calibration/lidar_camera_calibration_right.mat")

params = lidarParameters('OS1Gen1-32', 1024);
roi = [0, 10,-5, 5, -1, 2];

conPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive(params);
    l_bboxData = receive(l_detectionSubscriber);
    r_bboxData = receive(r_detectionSubscriber);

    roiPoints = getPointsInROI(receivedPoints, roi);
    nonGroundPoints = getNonGroundPoints(roiPoints, 0.06, [0,0,1], 30); % model, inlierIndices, outlierIndices, meanError
    denoisedPoints = pcdenoise(nonGroundPoints);

    l_bboxesCamera = getBboxesCamera(l_bboxData);
    r_bboxesCamera = getBboxesCamera(r_bboxData);
    [bboxesLidar,~,bboxesUsed] = bboxCameraToLidar(bboxesCamera, denoisedPoints, cameraParams, invert(tform), 'ClusterThreshold',0.2);
    [yBboxes, bBboxes, rBboxes] = classifyBboxLidar( ...
        bboxesLidar, ...
        [l_bboxData.Detections.Label r_bboxData.Detections.Label], ...
        bboxesUsed);

    view(conPyr, denoisedPoints)
    showShape("cuboid", ...
        [yBboxes; bBboxes; rBboxes], ...
        Parent=conPyr.Axes, ...
        Color="green", ...
        Opacity=0.5)
    drawnow

    yPos = yBboxes(:, 1:2);
    rPos = rBboxes(:, 1:2);
    bPos = bBboxes(:, 1:2);
end