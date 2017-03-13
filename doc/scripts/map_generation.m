% Title:        EE4308 Turtlebot Project
% File:         map_generation.m
% Date:         2017-02-13
% Author:       Preben Jensen Hoel (A0158996B) and
%               Paul-Edouard Sarlin (A0153124U)
% Description:  Matlab script used to plot an empty map.

clear all; close all;

w = 8; h = 8;

walls = [0.5,0;
         0.5,2;
         0.5,3;
         1.5,0;
         1.5,1;
         1.5,4;
         1.5,5;
         1.5,7;
         3.5,0;
         3.5,1;
         3.5,2;
         3.5,3;
         5.5,1;
         5.5,2;
         5.5,3;
         5.5,4;
         5.5,5;
         7.5,1;
         7.5,2;
         7.5,3;
         7.5,6;
         0,6.5;
         1,3.5;
         1,6.5;
         2,5.5;
         3,5.5;
         4,5.5;
         5,5.5;
         8,0.5;
         8,6.5];

% Print grid
x = -0.5:1:w+0.5; y = -0.5:1:h+0.5;
xv = repmat(x',1,2); yv = repmat([y(1),y(end)],length(x),1);
xh = repmat([x(1),x(end)],length(y),1); yh = repmat(y',1,2);
for i = 1:length(xv)
    line(xv(i,:),yv(i,:),'Color',[0.5,0.5,0.5]);
end
for i = 1:length(xh)
    line(xh(i,:),yh(i,:),'Color',[0.5,0.5,0.5]);
end

% Print walls
for i = 1:size(walls,1)
    hold on
    if mod(walls(i,1),1) ~= 0
        line([walls(i,1),walls(i,1)], [walls(i,2)-0.5, walls(i,2)+0.5],...
            'Color','k','LineWidth',4);
    else
        line([walls(i,1)-0.5,walls(i,1)+0.5], [walls(i,2), walls(i,2)],...
            'Color','k','LineWidth',4);
    end
end

% Setup plot
box on
axis equal
axis([-0.5 w+0.5 -0.5 h+0.5])
xticks(-1:w+1)
yticks(-1:h+1)
