clear; close all; clc;

bag=rosbag('2023-09-16-18-26-19.bag');

imageBag=select(bag,'Topic','/usb_cam/image_raw');
pcBag=select(bag, 'Topic','/ouster/points');

imageMsgs=readMessages(imageBag);
pcMsgs=readMessages(pcBag);

ts1 = timeseries(imageBag);
ts2 = timeseries(pcBag);

t1 = ts1.Time;
t2 = ts2.Time;

k = 1;

if size(t2,1) > size(t1,1)
    for i = 1:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2));
        if val <= 0.1 
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end

pcFilesPath = fullfile(pwd,'PointClouds');
imageFilesPath = fullfile(pwd,'Images');

if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end
for i = 1:length(idx)
    I = readImage(imageMsgs{idx(i,1)});
    pc = pointCloud(readXYZ(pcMsgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    pcwrite(pc,pcFileName);
end

