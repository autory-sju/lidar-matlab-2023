function pointsInROI = getPointsInROI(pointCloud, roi)
    
    % Get pointcloud in ROI(Region of interest)
    % Use findPointsInROI function

    % ---------------------------------------------------------------------

    index = findPointsInROI(pointCloud, roi);
    pointsInROI = select(pointCloud, index);
end