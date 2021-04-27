%%
clear; clc; clf;

q = [0 1.13 1.84];
qq = [0 q(2) q(3) (pi/2)-q(2)-q(3)];
base = eye(4);

robot = Dobot(base, qq);
hold on
% robot.model.teach();

table = PlotObject('table.ply', eye(4));