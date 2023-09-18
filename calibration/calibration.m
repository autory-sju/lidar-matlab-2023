imageFilesPath = fullfile(pwd,'Images');
pcFilesPath = fullfile(pwd,'PointClouds');
checkerSize=56; % 체크박스의 전체 사이즈 / 체크무늬 패턴의 수 , 1000(1m) / 8, 체크박스 1개 파라미터, mm단위로 입력한다.
padding=[0 0 0 0];
lidarCameraCalibrator(imageFilesPath,pcFilesPath,checkerSize,padding)