clear; close all; clc; rosshutdown;

rosinit();

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
params = lidarParameters('OS1Gen1-32', 1024);

cameraSubscriber1 = rossubscriber("/camera1/usb_cam1/image_raw");
cameraSubscriber2 = rossubscriber("/camera2/usb_cam2/image_raw");

load("./src/param/cam1.mat"); load("./src/param/cam2.mat");

lidarPlayer1 = pcplayer([0,20], [-10,10], [-1,2]);
lidarPlayer2 = pcplayer([0,20], [-10,10], [-1,2]);
mergePlayer = pcplayer([0,20], [-10,10], [-1,2]);

while true
    receivedPoints = lidarSubscriber.receive();


    rotationAngles = [0 0 -0.5];
    translation = [0 0 0];
    tform = rigidtform3d(rotationAngles,translation);
    receivedPoints = pctransform(receivedPoints,tform);


    cameraData1 = receive(cameraSubscriber1);
    cameraData2 = receive(cameraSubscriber2);

    frame1 = readImage(cameraData1);
    frame2 = readImage(cameraData2);

    fusedPt1 = fuseCameraToLidar(frame1, receivedPoints, cameraParams1, invert(tform1));
    fusedPt2 = fuseCameraToLidar(frame2, receivedPoints, cameraParams2, invert(tform2));

    view(lidarPlayer1, fusedPt1);
    view(lidarPlayer2, fusedPt2);

    merge = pcmerge(fusedPt1,fusedPt2,0.001);

    view(mergePlayer, merge);

    impts1 = projectLidarPointsOnImage(receivedPoints,cameraParams1,tform1);
    impts2 = projectLidarPointsOnImage(receivedPoints,cameraParams2,tform2);
    
    subplot(1,2,1);
    imshow(frame1)
    hold on
    plot(impts1(:,1),impts1(:,2),'.','Color','r')
    hold off

    subplot(1,2,2);
    imshow(frame2)
    hold on
    plot(impts2(:,1),impts2(:,2),'.','Color','r')
    hold off
end