function bboxesCamera = getBboxesCamera(bboxData)
    bboxesCnt = numel(bboxData.Detections);
    bboxesCamera = zeros(bboxesCnt, 4);
    for i = 1:bboxesCnt
        bboxesCamera(i, :) = [
            double(bboxData.Detections(i).Mask.Roi.X) ...
            double(bboxData.Detections(i).Mask.Roi.Y) ...
            double(bboxData.Detections(i).Mask.Roi.Width) ...
            double(bboxData.Detections(i).Mask.Roi.Height)];
    end
end