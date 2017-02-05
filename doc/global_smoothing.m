clear all; close all;

path = [0,0;
        0,1;
        1,1;
        1,2;
        2,2;
        3,2;
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
    
alpha = 0.7;
rate = 1;
tol = 1e-6;

smoothed = path;
err = tol;

while err >= tol
    err = 0;
    for i = 2:(size(path,1)-1)
        for j = 1:size(path,2)
            tmp = smoothed(i,j);
            smoothed(i,j) = smoothed(i,j) + rate*(alpha*(path(i,j)-smoothed(i,j)) ...
                + (1-alpha)*(smoothed(i+1,j)+smoothed(i-1,j)-2.*smoothed(i,j)));
            err = err + abs(tmp - smoothed(i,j));
        end
    end
end

plot(path(:,1),path(:,2),'-r*','LineWidth',1);
hold on;
plot(smoothed(:,1),smoothed(:,2),'-b*','LineWidth',1);
for i = 1:size(walls,1)
    hold on
    if mod(walls(i,1),1) ~= 0
        line([walls(i,1),walls(i,1)], [walls(i,2)-0.5, walls(i,2)+0.5],...
            'Color','k','LineWidth',4);
    else
        
    end
end