clear; close all; clc; rosshutdown;

rosinit("10.211.55.6");
params = lidarParameters('OS1Gen1-32', 1024);

lidarSubscriber = LidarSubscriber('/ouster/points', "DataFormat", "struct");
receivedPoints = lidarSubscriber.receive(params);