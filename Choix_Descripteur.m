clear all
close all
clc

%% Camera Calibration Parameters Loading

load('cameraParams_.mat')
% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams_);
% Visualize pattern locations
h=figure(2);showExtrinsics(cameraParams_,'CameraCentric');
% Create the camera intrinsics object using camera intrinsics
intrinsics = cameraParameters('IntrinsicMatrix', cameraParams_.IntrinsicMatrix);



%% Choice of descriptor and motion detection: 
% Read the image.
images = imageDatastore(".\Versailles\");
% Convert to gray scale and undistort.
I = rgb2gray(readimage(images, 10));
I = undistortImage(I, intrinsics);
Nb_img = size(images.Files,1);
numPoints = 2500;
time = 10;

% MSER
I_Points_MSER = detectMSERFeatures(I);
Nft_MSER = length(I_Points_MSER.Location);
fprintf('MSER : %d\n',Nft_MSER);
h_MSER=figure(3);imshow(I);hold on;scatter(I_Points_MSER.Location(:,1),I_Points_MSER.Location(:,2),200,'.w');
set(h_MSER,'WindowStyle','docked');hold off

% SURF
I_Points_SURF = detectSURFFeatures(I);
Nft_SURF = length(I_Points_SURF.Location);
fprintf('SURF : %d\n',Nft_SURF);
h_SURF=figure(4);imshow(I);hold on;scatter(I_Points_SURF.Location(:,1),I_Points_SURF.Location(:,2),200,'.w');
set(h_SURF,'WindowStyle','docked');hold off

% Harris
I_Points_Harris = detectHarrisFeatures(I);
Nft_Harris = length(I_Points_Harris.Location);
fprintf('Harris : %d\n',Nft_Harris);
h_Harris=figure(5);imshow(I);hold on;scatter(I_Points_Harris.Location(:,1),I_Points_Harris.Location(:,2),200,'.w');
set(h_Harris,'WindowStyle','docked');hold off

% ORB
I_Points_ORB = detectORBFeatures(I);
Nft_ORB = length(I_Points_ORB.Location);
fprintf('ORB : %d\n',Nft_ORB);
h_ORB=figure(6);imshow(I);hold on;scatter(I_Points_ORB.Location(:,1),I_Points_ORB.Location(:,2),200,'.w');
set(h_ORB,'WindowStyle','docked');hold off