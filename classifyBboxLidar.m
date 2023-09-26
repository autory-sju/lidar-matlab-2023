function [yBboxes, bBboxes, rBboxes] = classifyBboxLidar(bboxesLidar, label, boxesUsed)
    bboxesCnt = numel(boxesUsed);

    yTmp = zeros(bboxesCnt, 9);
    bTmp = zeros(bboxesCnt, 9);
    rTmp = zeros(bboxesCnt, 9);
    yCnt = 0;
    bCnt = 0;
    rCnt = 0;
    bbCnt = 0;

    for i = 1:bboxesCnt
        if boxesUsed(i)
            bbCnt = bbCnt + 1;
            switch label(i,:)
                case 'y_cone'
                    yCnt = yCnt + 1;
                    yTmp(yCnt, :) = bboxesLidar(bbCnt, :);
                case 'b_cone'
                    bCnt = bCnt + 1;
                    bTmp(bCnt, :) = bboxesLidar(bbCnt, :);
                case 'r_cone'
                    rCnt = rCnt + 1;
                    rTmp(rCnt, :) = bboxesLidar(bbCnt, :);
            end
        end
    end

    yBboxes =  yTmp(1:yCnt, :);
    bBboxes =  bTmp(1:bCnt, :);
    rBboxes =  rTmp(1:rCnt, :);
end