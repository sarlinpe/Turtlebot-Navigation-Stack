clear all; close all;

densify = true;
path = [0,0;
        0,1;
        1,1;
        1,2;
        2,2;
        2,3;%
        %3,2;%
        3,3;
        3,4;
        3,5;
        4,5;
        4,4];
walls = [0.5,0;
         0.5,2;
         0.5,3;
         1.5,0;
         1.5,1;
         3.5,0;
         3.5,1;
         3.5,2;
         3.5,3;
         3.5,4];

% Smoothing parameters
rate = 1;
tol = 1e-6;
if densify
    density = 4;
    alpha = 0.2;
else
    density = 1;
    alpha = 0.7;
end

% Print grid
x = -0.5:1:4.5; y = -0.5:1:5.5;
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
axis([-0.5 4.5 -0.5 5.5])
xticks(-1:4)

% Densify
dense = [];
for i = 1:(length(path)-1)
    for j = 0:(density-1)
        dense(end+1,:) = ((density-j)*path(i,:)+j*path(i+1,:))/density;
    end
end
dense(end+1,:) = path(end,:);

% Minimize cost function
smoothed = dense;
err = tol;
while err >= tol
    err = 0;
    for i = 2:(size(dense,1)-1)
        for j = 1:size(dense,2)
            tmp = smoothed(i,j);
            smoothed(i,j) = smoothed(i,j) + rate*(alpha*(dense(i,j)-smoothed(i,j)) ...
                + (1-alpha)*(smoothed(i+1,j)+smoothed(i-1,j)-2.*smoothed(i,j)));
            err = err + abs(tmp - smoothed(i,j));
        end
    end
end

% Print paths
hold on;
if densify
    plot(dense(:,1),dense(:,2),'-g*','LineWidth',1);
else
   plot(path(:,1),path(:,2),'-r*','LineWidth',1); 
end
hold on;
plot(smoothed(:,1),smoothed(:,2),'-b*','LineWidth',1);

