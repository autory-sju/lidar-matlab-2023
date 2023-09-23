clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
params = lidarParameters('OS1Gen1-32', 1024);

detectionSubscriber1 = rossubscriber("/yolov5/cob_detections_1");
detectionSubscriber2 = rossubscriber("/yolov5/cob_detections_2");

load("cameraParams.mat"); load("tform.mat");

roi = [0, 20, -10, 10, -1, 2];

% % conPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

yPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));
rPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));
bPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive(params);

    bboxData1 = receive(detectionSubscriber1);
    bboxData2 = receive(detectionSubscriber2);

    roiPoints = getPointsInROI(receivedPoints, roi);
    nonGroundPoints = getNonGroundPoints(roiPoints, 0.06, [0,0,1], 30);
    denoisedPoints = pcdenoise(nonGroundPoints);

    bboxesCamera1 = getBboxesCamera(bboxData1);
    bboxesCamera2 = getBboxesCamera(bboxData2);

    [bboxesLidar1,~,bboxesUsed1] = bboxCameraToLidar(bboxesCamera1, denoisedPoints, cameraParams1, invert(tform1), 'ClusterThreshold',0.2);
    [bboxesLidar2,~,bboxesUsed2] = bboxCameraToLidar(bboxesCamera2, denoisedPoints, cameraParams2, invert(tform2), 'ClusterThreshold',0.2);

    [yBboxes, bBboxes, rBboxes] = classifyBboxLidar( ...
        [bboxesLidar1; bboxesLidar2], ...
        vertcat(bboxData1.Detections.Label, bboxData2.Detections.Label), ...
        [bboxesUsed1; bboxesUsed2]);

    % view(conPyr, denoisedPoints)
    % showShape("cuboid", ...
    %     [yBboxes; bBboxes; rBboxes], ...
    %     Parent=conPyr.Axes, ...
    %     Color="green", ...
    %     Opacity=0.5)
    % drawnow

    view(rPyr, denoisedPoints)
    showShape("cuboid", ...
        rBboxes, ...
        Parent=rPyr.Axes, ...
        Color="red", ...
        Opacity=0.5)
    drawnow

    view(bPyr, denoisedPoints)
    showShape("cuboid", ...
        bBboxes, ...
        Parent=bPyr.Axes, ...
        Color="blue", ...
        Opacity=0.5)
    drawnow

    view(yPyr, denoisedPoints)
    showShape("cuboid", ...
        yBboxes, ...
        Parent=yPyr.Axes, ...
        Color="yellow", ...
        Opacity=0.5)
    drawnow

    yPos = yBboxes(:, 1:2);
    rPos = rBboxes(:, 1:2);
    bPos = bBboxes(:, 1:2);
end