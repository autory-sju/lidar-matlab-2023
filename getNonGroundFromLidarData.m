function nonGroundPoints = getNonGroundFromLidarData(pointCloud, params)
    organizedPcl  = pcorganize(pointCloud, params);
    groundPtsIdx = segmentGroundFromLidarData(organizedPcl, ...
        "ElevationAngleDelta",5, ...
        "InitialElevationAngle",15);
    
    nonGroundPoints = select(organizedPcl,~groundPtsIdx);
end