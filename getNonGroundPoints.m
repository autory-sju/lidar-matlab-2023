function nonGroundPoints = getNonGroundPoints(pointCloud, maxDistance, referenceVector, maxAngularDistance)
    
    % Get non-ground pointcloud by fit plane
    % Use MSAC(M-estimator SAmple Consensus) algorithm

    % ---------------------------------------------------------------------

    [~,~,outlierIndices] = ...
        pcfitplane(pointCloud, ...
        maxDistance, ...
        referenceVector, ...
        maxAngularDistance);

    nonGroundPoints = select(pointCloud,outlierIndices);
end