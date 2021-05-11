clear; clc;

A = rossubscriber('/camera/color/image_raw');
pause(1);
I = readImage(A.LatestMessage);
imshow(I)
