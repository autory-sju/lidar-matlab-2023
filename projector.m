clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
params = lidarParameters('OS1Gen1-32', 1024);
cameraSubscriber = rossubscriber("/usb_cam/image_raw");
load("calibration/camera_cali_result_9.16.20.mat")
load("calibration/lcc-2023-09-17-00.mat")

while true
    receivedPoints = lidarSubscriber.receive(params);

    imgMsg = receive(cameraSubscriber);
    img = readImage(imgMsg);
    imPts = projectLidarPointsOnImage( ...
        receivedPoints,cameraParams,tform);

    imshow(img)
    hold on
    plot(imPts(:,1),imPts(:,2),'.','Color','r')
    hold off
end