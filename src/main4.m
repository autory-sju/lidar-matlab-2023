clear; close all; clc; rosshutdown;

% rosinit("10.211.55.7");
rosinit("http://OMEN:11311/");

%% PERCEPTION
wDelay=0;

%% JUDGEMENT
pp=controllerPurePursuit;
pp.LookaheadDistance=4; % m
pp.DesiredLinearVelocity=0.3; % m/s
pp.MaxAngularVelocity = 5.0; % rad/s
waypointTreshold = 2;
% yaw = [0;0];
vehiclePose=[0,0,0];
gpsSub = rossubscriber('/utm');
utmSpeedSub = rossubscriber('/ublox_gps/fix_velocity');
%imuSub = rossubscriber('/imu');
prevw = 0;
waypoints = [];
worldWaypoints = [329929.930207119	4157686.02597486
    329929.772913363	4157684.81922735
    329931.839798109	4157683.03408197
    329933.516639571	4157681.62324906
    329934.745688611	4157680.78772302
    329936.257240858	4157680.36823407
    329938.132396477	4157680.44082542
    329939.405483156	4157680.89207237
    329940.512990153	4157681.45771797
    329942.171181676	4157682.15639086
    329943.479377933	4157682.59581852
    329944.466582523	4157682.89751370
    329945.710665041	4157683.22724869
    329946.747920601	4157683.38361049
    329947.852895968	4157683.39427684
    329948.705054369	4157683.14370689
    329949.788466636	4157682.53317933
    329950.701071548	4157681.78184252
    329951.349328015	4157681.06922282
    329951.915347883	4157680.22508007
    329952.594820500	4157678.87908452
    329952.919358287	4157677.89558016
    329953.088741788	4157677.10396496
    329953.250882033	4157676.39020262
    329953.410749369	4157675.56548044
    329953.532779084	4157674.61942629
    329953.659779202	4157673.48455945
    329953.774565688	4157672.61635817
    329953.895004462	4157671.59263215
    329954.004335984	4157670.45812725
    329954.104833186	4157669.32380332
    329954.253336176	4157667.94428195
    329954.372638659	4157666.86507602
    329954.471347674	4157666.07490859];
fig = figure();
pp.Waypoints = worldWaypoints;

posUtmData = receive(gpsSub);
veloUtmData = receive(utmSpeedSub);
velo = updateVehicleVelo(veloUtmData);

prevPos = [posUtmData.Pose.Position.X,posUtmData.Pose.Position.Y, 0];

%% Loop
while true
    %% PERCEPTION - LOOP


    if norm([posUtmData.Pose.Position.X,posUtmData.Pose.Position.Y]-prevPos(1:2)) > 1
        prevPos = [posUtmData.Pose.Position.X,posUtmData.Pose.Position.Y, vehiclePose(3)];
    end

    posUtmData = receive(gpsSub);
    veloUtmData = receive(utmSpeedSub);
    velo = updateVehicleVelo(veloUtmData);
    %imuData = receive(imuSub);


    vehiclePose = updateVehiclePose(posUtmData,prevPos);


    % if isempty(pp.Waypoints) || norm(worldWaypoints(end,:)-[vehiclePose(1), vehiclePose(2)]) < waypointTreshold  % Considering only x and y for the distance
    %disp("Make new waypoints");
    xlim([329900,329980]);
    ylim([4157660,4157700]);


    hold on
    scatter(worldWaypoints(:,1),worldWaypoints(:,2),'blue');
    scatter(vehiclePose(1), vehiclePose(2),'red');


    hold off

    [v, w] = pp(vehiclePose); % Pass the current vehicle pose to the path planner
w
    % if abs(prevw)>abs(w)
    %     w = -w;
    % end
    prevw = w;
    carL = 1.33;
    % wDelay = w * carL/v;

    if velo <= 0.1 || vehiclePose(3) == 0 || abs(w)<0.1
        wDelay = 0;
        % Using Target Speed
    else
        wDelay =w * carL/v;
    end
wDelay
    [pub, msg] = publish_twist_command(v, wDelay, velo, '/cmd_vel');
    send(pub, msg);

    % if abs(prevw)>abs(w)


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
function vehiclePose = updateVehiclePose(currentPosUtm, prevPose)
% Originally Imu gave values in degree but PP needs values in radian.

%raw_yaw = imu.Orientation.X;
% if raw_yaw>=90
%     yawD = raw_yaw-90;
% elseif raw_yaw>=0
%     yawD= raw_yaw - 90;
% elseif raw_yaw>=-90
%     yawD=  -90+raw_yaw;
% else
%     yawD= 270+raw_yaw;
% end
% yawRad = yawD * pi / 180;

% if raw_yaw >= 90
%     yawD = (180 - raw_yaw) + 90;
% elseif raw_yaw >= 0
%     yawD = -90 - (raw_yaw);
% elseif raw_yaw >= -90
%     yawD = -raw_yaw - 90;
% else
%     yawD = -(raw_yaw + 90);
% end
% yawRad = yawD * pi / 180;
currentPose = [currentPosUtm.Pose.Position.X,currentPosUtm.Pose.Position.Y];
nowVec = currentPose - prevPose(1:2);
xVec = [1,0];

if currentPose == prevPose(1:2)
    yawRad = 0;
elseif norm(currentPose-prevPose(1:2))>0.3
    if nowVec(2)<0
        yawRad =- acos(dot(nowVec,xVec)/norm(nowVec)/norm(xVec));
    else
        yawRad = acos(dot(nowVec,xVec)/norm(nowVec)/norm(xVec));
    end
else
    yawRad = prevPose(3);
end



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
        redCones(i,1)
        if redCones(i,1)<5
            redConeCnt = redConeCnt+1;
        end
        % if norm(redCones(i,:)) < 6
        %     redConeCnt = redConeCnt+1;
        % end
    end
    if redConeCnt>1
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
[innerM,~] = size(innerConePosition);
[outerM,~] = size(outerConePosition);
if innerM==1 | outerM==1
    waypoints = innerConePosition(1,:);
    waypoints = (waypoints + outerConePosition(1,:))/2;
else

    if innerM>outerM
        kockle_coords = zeros(innerM * 2,2); % initiate a P matrix consisting of inner and outer coordinates
        kockle_coords(1:2:2*innerM,:) = innerConePosition;
        % kockle_coords(2:2:2*outerM,:) = outerConePosition;
        % for i=2*outerM+2:2:2*innerM
        %     kockle_coords(i,:) = outerConePosition(outerM,:);
        % end
        for i = 1 : 1 :innerM
            kockle_coords(i*2,:) = outerConePosition(1,:)*(innerM-i)/(innerM-1) +outerConePosition(outerM,:)*(i-1)/(innerM-1);
            kockle_coords(i*2,2) = kockle_coords(i*2,2) +0.1*i;

        end

    elseif innerM<outerM
        kockle_coords = zeros(outerM * 2,2); % initiate a P matrix consisting of inner and outer coordinates
        % kockle_coords(1:2:2*innerM,:) = innerConePosition;
        kockle_coords(2:2:2*outerM,:) = outerConePosition;
        % for i=2*innerM+1:2:2*outerM
        %     kockle_coords(i,:) = innerConePosition(innerM,:);
        % end
        for i = 1 : 1 :outerM
            kockle_coords(i*2-1,:) = innerConePosition(1,:)*(outerM-i)/(outerM-1) +innerConePosition(innerM,:)*(i-1)/(outerM-1);
            kockle_coords(i*2-1,2) = kockle_coords(i*2-1,2) +0.1*i;

        end


    else
        kockle_coords = zeros(innerM * 2,2); % initiate a P matrix consisting of inner and outer coordinates
        kockle_coords(1:2:2*innerM,:) = innerConePosition;
        kockle_coords(2:2:2*innerM,:) = outerConePosition;
    end
    scatter(kockle_coords(:,1),kockle_coords(:,2),'black');



    %%%%%%%%%%%%%%%%%%
    %[m,nc] = size(innerConePosition); % size of the inner/outer cone positions data
    %kockle_coords = zeros(m * 2,nc); % initiate a P matrix consisting of inner and outer coordinates
    %kockle_coords(1:2:2*m,:) = innerConePosition;
    %kockle_coords(2:2:2*m,:) = outerConePosition; % merge the inner and outer coordinates with alternate values



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
end

function [pub, msg] = publish_twist_command(v, w, curVelo, topicName)
pub = rospublisher(topicName, 'geometry_msgs/Twist','DataFormat','struct');
msg = rosmessage(pub);
msg.Linear.X = v;
msg.Linear.Z = curVelo;
msg.Angular.Z = w;
end