clear; close all; clc; rosshutdown;

% rosinit("10.211.55.7");
rosinit("http://OMEN:11311/");

%% PERCEPTION
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


%% JUDGEMENT
pp=controllerPurePursuit;
pp.LookaheadDistance=5; % m
pp.DesiredLinearVelocity=1; % m/s
pp.MaxAngularVelocity = 2.0; % rad/s
waypointTreshold = 2;
% yaw = [0;0];
gpsSub = rossubscriber('/utm');
utmSpeedSub = rossubscriber('/ublox_gps/fix_velocity');
imuSub = rossubscriber('/imu');
prevw = 0;
waypoints = [];

%% Loop
while true
    %% PERCEPTION - LOOP

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

    %% JUDGEMENT - LOOP
    redCones = rPos;

    % Emergency Stop by red cones for Brake test
    while redConeBrake(redCones) == 1
        [pub, msg] = publish_twist_command(0, 0,velo, '/cmd_vel');
        send(pub, msg);
    end

    posUtmData = receive(gpsSub);
    veloUtmData = receive(utmSpeedSub);
    velo = updateVehicleVelo(veloUtmData);
    imuData = receive(imuSub);


    vehiclePose = updateVehiclePose(posUtmData,imuData);

    if isempty(pp.Waypoints) || norm(worldWaypoints(end,:)-[vehiclePose(1), vehiclePose(2)]) < waypointTreshold  % Considering only x and y for the distance
        disp("Make new waypoints");

        try
            innerConePosition = bPos;
            outerConePosition = yPos;
            % For watching Cone perception
            hold off;
            scatter(innerConePosition(:,1),innerConePosition(:,2),'blue');
            hold on;
            scatter(outerConePosition(:,1),outerConePosition(:,2),'red');

            % MATCHING BOTH SIDE CONE LENGTH
            [innerConePosition, outerConePosition] = match_array_lengths(innerConePosition, outerConePosition);
            waypoints = generate_waypoints_del(innerConePosition, outerConePosition);

            worldWaypoints = transformWaypointsToOdom(waypoints, vehiclePose);

            pp.Waypoints = worldWaypoints;
        catch
            disp("Fail to make new waypoints");
            continue; % 다음 while문 반복으로 넘어감
        end
    end

    [v, w] = pp(vehiclePose) % Pass the current vehicle pose to the path planner

    if abs(prevw)>abs(w)
        w = -w;
    end
    prevw = w;
    carL = 1.33;
    %    wDelay = w*400/pi/1e6*30*carL/v;
    wDelay = w * carL/v;

    [pub, msg] = publish_twist_command(v, wDelay, velo, '/cmd_vel');
    send(pub, msg);

    % 종방향 속도, 횡방향 각속도
    % tractive_control = v;
    % steering_control = w;

end

%% JUDGEMENT - Util, Func

% getting velo
function currentVelo = updateVehicleVelo(veloUtmData)

xUtmVelo = veloUtmData.Twist.Twist.Linear.X;
yUtmVelo = veloUtmData.Twist.Twist.Linear.Y;
utmVelo = sqrt(xUtmVelo^2 + yUtmVelo^2);

currentVelo = utmVelo;
end

% utmX, utmY, yaw
function vehiclePose = updateVehiclePose(currentPosUtm, imu)
% Originally Imu gave values in degree but PP needs values in radian.

raw_yaw = imu.Orientation.X;
if raw_yaw>=90
    yawD = raw_yaw-90;
elseif raw_yaw>=0
    yawD= raw_yaw - 90;
elseif raw_yaw>=-90
    yawD=  -90+raw_yaw;
else
    yawD= 270+raw_yaw;
end
yawRad = yawD * pi / 180;

vehiclePose=[currentPosUtm.Pose.Position.X,currentPosUtm.Pose.Position.Y,yawRad];
end

% redConeBrake (for brake test)
function isStop = redConeBrake(redCones)
isStop = 0;

if size(redCones,1) ~= 0
    redConeCnt = 0;
    % for every red cones detecte
    for i=1:1:size(redCones,1)
        % distance between one of red cone is under 5meter
        if redCones(i,1)<5
            redConeCnt = redConeCnt+1;
        end
        % if norm(redCones(i,:)) < 6
        %     redConeCnt = redConeCnt+1;
        % end
    end
    if redConeCnt>2
        isStop = 1;
    end
end
end

% Convert Car based waypoint to World based waypoints
function odomWaypoints = transformWaypointsToOdom(waypoints, vehiclePoseInOdom)
% Initialize transformed waypoints
odomWaypoints = zeros(size(waypoints));

% Extract the vehicle's yaw angle
theta = vehiclePoseInOdom(3);

% Create the 2D rotation matrix
R = [cos(theta), -sin(theta);
    sin(theta), cos(theta)];

% Transform each waypoint
for i = 1:size(waypoints,1)
    % Rotate the waypoint considering the vehicle's yaw
    rotatedPoint = R * waypoints(i,:)';

    % Translate considering the vehicle's position in the odom frame
    odomWaypoints(i,:) = rotatedPoint' + vehiclePoseInOdom(1:2);
end
end


% match both side rubber cone
function [out1, out2] = match_array_lengths(arr1, arr2)
len1 = size(arr1, 1); % Get the number of rows
len2 = size(arr2, 1); % Get the number of rows

if len1 > len2
    out1 = arr1(1:len2, :); % Keep only the first len2 rows
    out2 = arr2;
elseif len2 > len1
    out1 = arr1;
    out2 = arr2(1:len1, :); % Keep only the first len1 rows
else
    out1 = arr1;
    out2 = arr2;
end
end


function waypoints = generate_waypoints_del(innerConePosition, outerConePosition)
[m,nc] = size(innerConePosition); % size of the inner/outer cone positions data
kockle_coords = zeros(m * 2,nc); % initiate a P matrix consisting of inner and outer coordinates
kockle_coords(1:2:2*m,:) = innerConePosition;
kockle_coords(2:2:2*m,:) = outerConePosition; % merge the inner and outer coordinates with alternate values
xp = []; % create an empty numeric xp vector to store the planned x coordinates after each iteration
yp = [];

interv=size(innerConePosition,1)*2;
%step 1 : delaunay triangulation
tri=delaunayTriangulation(kockle_coords);
pl=tri.Points;
cl=tri.ConnectivityList;
[mc, nc]=size(pl);

% inner and outer constraints when the interval is even
if rem(interv,2) == 0
    cIn = [2 1;(1:2:mc-3)' (3:2:(mc))'; (mc-1) mc];
    cOut = [(2:2:(mc-2))' (4:2:mc)'];
else
    % inner and outer constraints when the interval is odd
    cIn = [2 1;(1:2:mc-2)' (3:2:(mc))'; (mc-1) mc];
    cOut = [(2:2:(mc-2))' (4:2:mc)'];
end

%step 2 : 외부 삼각형 거
C = [cIn;cOut];
TR=delaunayTriangulation(pl,C);
% TRC=TR.ConnectivityList;
TL=isInterior(TR);
TC =TR.ConnectivityList(TL,:);
[~, pt]=sort(sum(TC,2));
TS=TC(pt,:);
TO=triangulation(TS,pl);

%step 3 : 중간 waypoint 생성
xPo=TO.Points(:,1);
yPo=TO.Points(:,2);
E=edges(TO);
iseven=rem(E,2)==0;
Eeven=E(any(iseven,2),:);
isodd=rem(Eeven,2)~=0;
Eodd=Eeven(any(isodd,2),:);
xmp=((xPo((Eodd(:,1))) + xPo((Eodd(:,2))))/2);
ymp=((yPo((Eodd(:,1))) + yPo((Eodd(:,2))))/2);
Pmp=[xmp ymp];
waypoints = Pmp;

end

function [pub, msg] = publish_twist_command(v, w, curVelo, topicName)
pub = rospublisher(topicName, 'geometry_msgs/Twist','DataFormat','struct');
msg = rosmessage(pub);
msg.Linear.X = v;
msg.Linear.Z = curVelo;
msg.Angular.Z = w;
end