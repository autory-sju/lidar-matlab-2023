clear; close all; clc; rosshutdown;

rosinit("10.211.55.7");

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
% params = lidarParameters('OS1Gen1-32', 1024);

detectionSubscriber1 = rossubscriber("/yolov5/cob_detections_1");
detectionSubscriber2 = rossubscriber("/yolov5/cob_detections_2");

load("./src/param/cameraParams.mat"); load("./src/param/tform.mat");

roi = [0, 20, -10, 10, -1, 2];

coneW = 400 * 0.001;
coneH = 700 * 0.001;
resoultionHorizontal = 45 / (32-1);
resoultionVertical = 360 / 1024;

filter = @(x, y) coneW*coneH / (8 * norm([x, y])^2 * tand(resoultionVertical/2) * tand(resoultionHorizontal/2));


conPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive();

    bboxData1 = receive(detectionSubscriber1);
    bboxData2 = receive(detectionSubscriber2);

    roiPoints = getPointsInROI(receivedPoints, roi);
    nonGroundPoints = getNonGroundSMRF(roiPoints);

    [labels,numClusters] = pcsegdist(nonGroundPoints,0.3);

    mergedPoints = pointCloud([0,0,0]);

    for i = 1:numClusters
        clusterIndices = find(labels == i);
        clusterCloud = select(nonGroundPoints, clusterIndices);
        clusterCenter(:) = mean(clusterCloud.Location);

        expectedPointCount = filter(clusterCenter(1), clusterCenter(2));
        
        if clusterCloud.Count > expectedPointCount * 1.4
            continue;
        end

        reconstructedPoints = getPointsInCylinder(clusterCenter, roiPoints);

        if reconstructedPoints.Count < expectedPointCount * 0.4
            continue;
        end

        mergedPoints = pcmerge(mergedPoints, reconstructedPoints, 0.01);
    end

    bboxesCamera1 = getBboxesCamera(bboxData1);
    bboxesCamera2 = getBboxesCamera(bboxData2);

    [bboxesLidar1,~,bboxesUsed1] = bboxCameraToLidar(bboxesCamera1, mergedPoints, cameraParams1, invert(tform1), 'ClusterThreshold',0.3);
    [bboxesLidar2,~,bboxesUsed2] = bboxCameraToLidar(bboxesCamera2, mergedPoints, cameraParams2, invert(tform2), 'ClusterThreshold',0.3);

    [yBboxes, bBboxes, rBboxes] = classifyBboxLidar( ...
        [bboxesLidar1; bboxesLidar2], ...
        vertcat(bboxData1.Detections.Label, bboxData2.Detections.Label), ...
        [bboxesUsed1; bboxesUsed2]);

    view(conPyr, mergedPoints)
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