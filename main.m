clear all
close all
clc

% Chargement des paramètres de la camera apres calibration

load('cameraParams_.mat')
% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams_);
% Visualize pattern locations
h=figure(2);showExtrinsics(cameraParams_,'CameraCentric');
% Create the camera intrinsics object using camera intrinsics
intrinsics = cameraParameters('IntrinsicMatrix', cameraParams_.IntrinsicMatrix);


% Detection du mouvement
time = 10;
images = imageDatastore(".\Versailles\");
load("cameraParams_.mat")
Nb_img = size(images.Files,1);

for t = 1:time
    % Create an empty viewSet object to manage the data associated with each view.
    vSet = viewSet;
    % Read the image.
    Irgb = readimage(images, 1);
    % Convert to gray scale and undistort.
    prevI = undistortImage(rgb2gray(Irgb), intrinsics);
    % Detect features.
    prevPoints = detectSURFFeatures(prevI,'NumOctave',4);
    % Select a subset of features, uniformly distributed throughout the image.
    numPoints = 5000 ;
    prevPoints = selectUniform(prevPoints, numPoints, size(prevI));
    % Extract features. Using 'Upright' features improves matching quality if 
    % the camera motion involves little or no in-plane rotation.
    prevFeatures = extractFeatures(prevI, prevPoints, 'Upright', true);
    % Add the first view. Place the camera associated with the first view
    % at the origin, oriented along the Z-axis.
    viewId = 1;
    vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
        'Location', [0 0 0]);
    % Read second image.
    viewId = 2;
    Irgb = readimage(images, viewId);
    % Convert to gray scale and undistort.
    I = undistortImage(rgb2gray(Irgb), intrinsics);
    % Detect features.
    currPoints   = detectSURFFeatures(I,'NumOctave',4);
    % Select a subset of features, uniformly distributed throughout the image.
    currPoints   = selectUniform(currPoints, numPoints, size(I));
    % Extract features. Using 'Upright' features improves matching quality if 
    % the camera motion involves little or no in-plane rotation.
    currFeatures = extractFeatures(I, currPoints, 'Upright', true);
    % Match features between the previous and current image.
    indexPairs = matchFeatures(prevFeatures, currFeatures,'MatchThreshold',5);
    % Estimate the pose of the current view relative to the previous view.
    [orient, loc, inlierIdx] = helperEstimateRelativePose(...
        prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), intrinsics);
    % Exclude epipolar outliers.
    indexPairs = indexPairs(inlierIdx, :);
    % Add the current view to the view set.
    vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
        'Location', loc);
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);

    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;
    
    for viewId = 3:Nb_img
        % Read the image.
        Irgb = readimage(images, viewId);
        % Convert to gray scale and undistort.
        I = undistortImage(rgb2gray(Irgb), intrinsics);
        % Detect features.
        currPoints   = detectSURFFeatures(I,'NumOctave',4);
        % Select a subset of features, uniformly distributed throughout the image.
        currPoints   = selectUniform(currPoints, numPoints, size(I));
        % Extract features. Using 'Upright' features improves matching quality if 
        % the camera motion involves little or no in-plane rotation.
        currFeatures = extractFeatures(I, currPoints, 'Upright', true);
        % Match features between the previous and current image.
        indexPairs = matchFeatures(prevFeatures, currFeatures, 'Unique', true);
        % Eliminate outliers from feature matches.
        inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)),...
            currPoints(indexPairs(:, 2)), intrinsics);
        indexPairs = indexPairs(inlierIdx, :);
        % Triangulate points from the previous two views, and find the 
        % corresponding points in the current view.
        [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,...
            intrinsics, indexPairs, currPoints);

        % Since RANSAC involves a stochastic process, it may sometimes not
        % reach the desired confidence level and exceed maximum number of
        % trials. Disable the warning when that happens since the outcomes are
        % still valid.

        warningstate = warning('off','vision:ransac:maxTrialsReached');
        % Estimate the world camera pose for the current view.
        [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, intrinsics);
        % Restore the original warning state
        warning(warningstate)
        % Add the current view to the view set.
        vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
            'Location', loc);
        % Store the point matches between the previous and the current views.
        vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
        tracks = findTracks(vSet); % Find point tracks spanning multiple views.
        camPoses = poses(vSet);    % Get camera poses for all views.
        % Triangulate initial locations for the 3-D world points.
        xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
        [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
            intrinsics, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
            'RelativeTolerance', 1e-9, 'MaxIterations', 300);
        vSet = updateView(vSet, camPoses); % Update view set.
        
        prevI = I;
        prevFeatures = currFeatures;
        prevPoints   = currPoints;
    end
    % Geu trajectory
    for i=1:Nb_img
        G(i,:,t)=poses(vSet).Location{i};
    end
    clearvars -except G time intrinsics Nb_img images
    
end
% Plot trajectory
h=figure(8);set(h,'WindowStyle','docked')
for h = 1:time
     plot(G(:,1,h),G(:,2,h))
 hold on
end