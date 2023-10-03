clear; close all; clc; 

ptCloud = pcread('Rosbag_x2023_09_23_16_58_43/600.pcd');

params = lidarParameters('OS1Gen1-32', 1024);

coneW = 400 * 0.001;
coneH = 700 * 0.001;
resoultionHorizontal = 45 / (32-1);
resoultionVertical = 360 / 1024;

filter = @(x, y) coneW*coneH / (8 * norm([x, y])^2 * tand(resoultionVertical/2) * tand(resoultionHorizontal/2));

roi = [0, 20, -10, 10, -1, 2];

roiPoints = getPointsInROI(ptCloud, roi);

figure(OuterPosition=[0,0,560,600])
pcshow(roiPoints);

nonGroundPoints = getNonGroundSMRF(roiPoints);

[labels,numClusters] = pcsegdist(nonGroundPoints,0.3);

figure(OuterPosition=[560,0,560,600])
pcshow(nonGroundPoints.Location,labels)
colormap(hsv(numClusters))


mergedPoints = pointCloud([0,0,0]);

for i = 1:numClusters
    % clustering
    clusterIndices = find(labels == i);
    clusterCloud = select(nonGroundPoints, clusterIndices);
    clusterCenter(:) = mean(clusterCloud.Location);

    % compute count for filtering
    expectedPointCount = filter(clusterCenter(1), clusterCenter(2));
    % pre-filtering
    if clusterCloud.Count > expectedPointCount * 1.4
        continue;
    end

    % reconstruction
    indicies = findPointsInCylinder( ...
        roiPoints,0.2,Center=clusterCenter);
    pointsInCylinder = select(roiPoints, indicies);

    % filtering
    if pointsInCylinder.Count < expectedPointCount * 0.4
        continue;
    end

    mergedPoints = pcmerge(mergedPoints, pointsInCylinder, 0.01);
end


figure(OuterPosition=[560*2,0,560,600])
pcshow(mergedPoints);

