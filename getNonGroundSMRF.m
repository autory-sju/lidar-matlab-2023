function nonGroundPtCloud = getNonGroundSMRF(pointCloud)
    [~,nonGroundPtCloud,~] = segmentGroundSMRF(pointCloud);
end