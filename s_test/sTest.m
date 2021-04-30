%% Begin
%Task need to be done:
% - Make the robot retreat from a "simulated safety symbol" using 
% VISUAL SERVOVING and RMRC
% - Helping team with "GUI"
% - Make sure understand LAB 8 + LAB 9 

clear all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf

%% Set up environment
% this section follow the test file by Minh
q = [0 1.13 1.84];
qq = [0 q(2) q(3) (pi/2)-q(2)-q(3)];
base = eye(4);
dobot = Dobot(base, qq);

% try to use my environment 
% enableTheEnvironment();
% hold on; %uncmt this if he enableTheEnvironment not on
surf([-1.7,-1.7;-1.7,-1.7],[-3,3;-3,3],[1.7,1.7;0,0],'CData',imread('s_Robotics.jpg'),'FaceColor','texturemap');
surf([-2,-2;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('s_concrete.jpg'),'FaceColor','texturemap');
axis equal;

