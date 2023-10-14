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
worldWaypoints = [
302480.696083935	4123671.63414136
302480.967446105	4123671.47232571
302481.300386817	4123671.28685679
302481.817802059	4123671.01933197
302482.194839370	4123670.82172332
302482.606579561	4123670.58999160
302483.045456538	4123670.37982574
302483.440236967	4123670.18179966
302483.896334875	4123669.94902429
302484.352432809	4123669.71624894
302484.771478138	4123669.41773238
302485.134161307	4123668.98731587
302485.467096921	4123668.42437318
302485.727232741	4123667.78542818
302485.847773798	4123667.24968737
302485.895515024	4123666.63794425
302485.854279827	4123666.01719250
302485.748855219	4123665.30913365
302485.753283106	4123664.74281838
302485.755622340	4123664.08773482
302485.755089679	4123663.31059487
302485.759517580	4123662.74427960
302485.957030696	4123662.08460374
302486.514621860	4123661.26102505
302487.142146322	4123660.76886627
302487.809595363	4123660.46450518
302488.498703705	4123660.32616720
302489.280443631	4123660.35218243
302490.282671729	4123660.69497308
302490.968918262	4123661.18952677
302491.600374010	4123661.99623064
302491.921064724	4123662.79914421
302492.001440650	4123663.57440552
302491.870988276	4123664.44344474
302491.534140946	4123665.21742056
302490.939235940	4123665.96416161
302490.451306559	4123666.35311602
302489.811255755	4123666.69013881
302489.231478196	4123666.94802824
302488.425218049	4123667.38888147
302487.619480144	4123667.85192685
302487.050140742	4123668.17618391
302486.463319398	4123668.51195453];
fig = figure();
pp.Waypoints = worldWaypoints;

posUtmData = receive(gpsSub);
veloUtmData = receive(utmSpeedSub);
velo = updateVehicleVelo(veloUtmData);

vehiclePose = [posUtmData.Pose.Position.X,posUtmData.Pose.Position.Y, 0];
startPoint = [posUtmData.Pose.Position.X,posUtmData.Pose.Position.Y, 0];

%% Loop
while true
    %% PERCEPTION - LOOP



    posUtmData = receive(gpsSub);
    veloUtmData = receive(utmSpeedSub);
    velo = updateVehicleVelo(veloUtmData);
    %imuData = receive(imuSub);


    vehiclePose = updateVehiclePose(posUtmData,vehiclePose,startPoint);


    % if isempty(pp.Waypoints) || norm(worldWaypoints(end,:)-[vehiclePose(1), vehiclePose(2)]) < waypointTreshold  % Considering only x and y for the distance
    %disp("Make new waypoints");
    xlim([worldWaypoints(1,1)-50,worldWaypoints(1,1)+50]);
    ylim([worldWaypoints(1,2)-50,worldWaypoints(1,2)+50]);


    hold on
    scatter(worldWaypoints(:,1),worldWaypoints(:,2),'blue');
    scatter(vehiclePose(1), vehiclePose(2),'red');


    hold off

    [v, w] = pp(vehiclePose) % Pass the current vehicle pose to the path planner
    w;
    % if abs(prevw)>abs(w)
    %     w = -w;
    % end
    %prevw = w;
    carL = 1.33;
    % wDelay = w * carL/v;

    if velo <= 0.05 || vehiclePose(3) == 0 || abs(w)<0.01
        wDelay = 0;
        % Using Target Speed
    else
        wDelay =w * carL/v*5;
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
function vehiclePose = updateVehiclePose(currentPosUtm, prevPose,startPoint)

currentPose = [currentPosUtm.Pose.Position.X,currentPosUtm.Pose.Position.Y];
nowVec = currentPose - prevPose(1:2);
xVec = [1,0];

if norm(currentPose - startPoint(1:2))<0.3
    yawRad = 0;
elseif norm(currentPose - prevPose(1:2))>0.05
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