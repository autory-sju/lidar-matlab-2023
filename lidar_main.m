clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
params = lidarParameters('OS1Gen1-32', 1024);

% camera subscriber ------------ !need refactor ++++++++++++++
detectionSubscriber = rossubscriber("/yolov5/cob_detections");

cameraSubscriber = rossubscriber("/usb_cam/image_raw");

load("calibration/camera_cali_result_9.16.20.mat")
load("calibration/lcc-2023-09-17-00.mat")

roi = [0, 20,-5, 5, -1, 2];

initPlayer

% ++++++++++++++
yPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));
bPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));
rPyr = pcplayer(roi(1:2), roi(3:4), roi(5:6));

while true
    receivedPoints = lidarSubscriber.receive(params);
    % ++++++++++++++
    imgMsg = receive(cameraSubscriber);
    img = readImage(imgMsg);
    imPts = projectLidarPointsOnImage( ...
        receivedPoints,cameraParams,tform);
    imshow(img)
    hold on
    plot(imPts(:,1),imPts(:,2),'.','Color','r')
    hold off

    bboxData = receive(detectionSubscriber);

    roiPoints = getPointsInROI(receivedPoints, roi);
    view(roiPyr, roiPoints);

    nonGroundPoints = getNonGroundPoints(roiPoints, 0.06, [0,0,1], 30);
    % view(grmPyr, nonGroundPoints);

    denoisedPoints = pcdenoise(nonGroundPoints);
    view(denPyr, denoisedPoints);

    % ++++++++++++++
    bboxsCnt = numel(bboxData.Detections);
    bboxsCamera = zeros(bboxsCnt, 4);
    for i = 1:bboxsCnt
        bboxsCamera(i, :) = [
            double(bboxData.Detections(i).Mask.Roi.X) ...
            double(bboxData.Detections(i).Mask.Roi.Y) ...
            double(bboxData.Detections(i).Mask.Roi.Width) ...
            double(bboxData.Detections(i).Mask.Roi.Height)];
    end
    [bboxesLidar,~,boxesUsed] = bboxCameraToLidar(bboxsCamera, denoisedPoints, cameraParams, invert(tform), 'ClusterThreshold',0.3);
    
    yTmp = zeros(bboxsCnt, 9);
    bTmp = zeros(bboxsCnt, 9);
    rTmp = zeros(bboxsCnt, 9);
    yCnt = 0;
    bCnt = 0;
    rCnt = 0;
    bbCnt = 0;

    for i = 1:bboxsCnt
        if boxesUsed(i)
            bbCnt = bbCnt + 1;
            switch bboxData.Detections(i).Label
                case 'y_cone'
                    yCnt = yCnt + 1;
                    yTmp(yCnt, :) = bboxesLidar(bbCnt, :);
                case 'b_cone'
                    bCnt = bCnt + 1;
                    bTmp(bCnt, :) = bboxesLidar(bbCnt, :);
                case 'r_cone'
                    rCnt = rCnt + 1;
                    rTmp(rCnt, :) = bboxesLidar(bbCnt, :);
            end
        end
    end

    yBbox =  yTmp(1:yCnt, :);
    bBbox =  bTmp(1:bCnt, :);
    rBbox =  rTmp(1:rCnt, :);

    view(yPyr, denoisedPoints)
    view(bPyr, denoisedPoints)
    view(rPyr, denoisedPoints)
    showShape("cuboid",yBbox, ...
        Parent=yPyr.Axes, ...
        Color="yellow", ...
        Opacity=0.5)
    showShape("cuboid",bBbox, ...
        Parent=bPyr.Axes, ...
        Color="blue", ...
        Opacity=0.5)
    showShape("cuboid",rBbox, ...
        Parent=rPyr.Axes, ...
        Color="red", ...
        Opacity=0.5)

    yLoc = yBbox(:, 1:2);
    bLoc = bBbox(:, 1:2);
    rLoc = rBbox(:, 1:2);
end


