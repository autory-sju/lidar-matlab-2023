classdef LidarSubscriber 
    
    % Clsss for use LiDAR with ROS
    % Initialize LiDAR subscriber connected to ROS

    % ---------------------------------------------------------------------

    properties
        Value
    end

    methods

        %LidarSubscriber
        %   Initialize LiDAR subscriber
        %   Get ros.Subscriber object via rossubscriber()
        function obj = LidarSubscriber(topic, varargin)
            obj.Value = rossubscriber(topic, varargin{:});
        end

        %receive
        %   Return pointcloud from LiDAR
        %   Receive and organize points
        function receivedPcl = receive(obj, params)
           lidarData = receive(obj.Value);
           xyzData = rosReadXYZ(lidarData);
           rawPcl = pointCloud(xyzData);
           receivedPcl = pcorganize(rawPcl,params);
        end
    end
end