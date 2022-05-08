% Camera Calibration Parameters Loading
load('cameraParams_.mat')
% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams_);
% Visualize pattern locations
h=figure(2);showExtrinsics(cameraParams_,'CameraCentric');
% Create the camera intrinsics object using camera intrinsics
intrinsics = cameraParameters('IntrinsicMatrix', cameraParams_.IntrinsicMatrix);



% Read the image.
images = imageDatastore(".\Versailles\");
Nb_img = size(images.Files,1);
Irgb = readimage(images, 1);
% Convert to gray scale and undistort.
I1 = undistortImage(rgb2gray(Irgb), intrinsics);

% Detect features. 
I1_Points = detectSURFFeatures(I1,'NumOctave',4);
% Select a subset of features, uniformly distributed throughout the image.
numPoints = 2500;
I1_Points = selectUniform(I1_Points, numPoints, size(I1));
% Extract features. Using 'Upright' features improves matching quality if 
% the camera motion involves little or no in-plane rotation.
I1_Features = extractFeatures(I1, I1_Points, 'Upright', true);

h=figure(7); ax = axes;set(h,'WindowStyle','docked')

for viewId = 2:Nb_img
    % Read the image.
    Irgb = readimage(images, viewId);
    % Convert to gray scale and undistort.
    I2 = undistortImage(rgb2gray(Irgb), intrinsics);
    % Detect features.
    I2_Points = detectSURFFeatures(I2,'NumOctave',4);
    I2_Features = extractFeatures(I2, I2_Points, 'Upright', true);
    % Match features between the previous and current image.
    indexPairs = matchFeatures(I1_Features,I2_Features, 'Unique', true);
    matchedPoints1 = I1_Points(indexPairs(:,1));
    matchedPoints2 = I2_Points(indexPairs(:,2));
    % Estimate the essential matrix. 
    [E,inliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2);
    % Get the epipolar inliers.
    inlierPoints1 = matchedPoints1(inliers).Location;
    inlierPoints2 = matchedPoints2(inliers).Location;
    
    % Plot trajectory
    lineX = [inlierPoints1(:,1)'; inlierPoints2(:,1)'];
    numPts = numel(lineX);
    lineX = [lineX; NaN(1,numPts/2)];
    lineY = [inlierPoints1(:,2)'; inlierPoints2(:,2)'];
    lineY = [lineY; NaN(1,numPts/2)];
    plot(lineX(:), lineY(:),'-k'); % line
    hold on
    
    I1 = I2;
    I1_Features = I2_Features;
    I1_Points   = I2_Points;
end