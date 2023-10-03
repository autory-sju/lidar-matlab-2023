function nonGroundPoints = getNonGroundPcFitPlane(pointCloud, maxDistance, referenceVector, maxAngularDistance)
    
    % Get non-ground pointcloud by fit plane
    % Use MSAC(M-estimator SAmple Consensus) algorithm
    % model = pcfitplane(ptCloudIn,maxDistance)
    % model = pcfitplane(ptCloudIn,maxDistance,referenceVector)
    % model = pcfitplane(ptCloudIn,maxDistance,referenceVector,maxAngularDistance)

    % ---------------------------------------------------------------------

    [~,~,outlierIndices] = ...
        pcfitplane(pointCloud, ...
        maxDistance, ...
        referenceVector, ...
        maxAngularDistance);

    nonGroundPoints = select(pointCloud,outlierIndices);
end