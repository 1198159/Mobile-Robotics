clc
clear all
close all

tmap1 = [9	5	5	3;	
14	15	15	10;
15	9	5	6;
13	6	15	15];


tmap2 = [11 15 15 11;
8 5 5 2;
10 15 15 10;
14 13 5 6];
function wallmap = top2walls(tmap)
    w = width(tmap);
    h = height(tmap);
    wallmap = zeros(h + h + 1, w + 1);
    for i = 1:h
        for j = 1:w
            wallmap(2*i -1, j) = bitor(wallmap(2*i - 1, j), bitshift(bitshift(tmap(i, j), 63), -63)); %up
            wallmap(2*i, j + 1) = bitor(wallmap(2*i, j + 1), bitshift(bitshift(bitshift(tmap(i, j), -1), 63), -63)); %right
            wallmap(2*i + 1, j) = bitor(wallmap(2*i + 1, j), bitshift(bitshift(bitshift(tmap(i, j), -2), 63), -63)); %down
            wallmap(2*i, j) = bitor(wallmap(2*i, j), bitshift(bitshift(bitshift(tmap(i, j), -3), 63), -63)); %left

        end
    end
end

function printwallmap(wallmap)
    format compact
    w = width(wallmap);
    h = height(wallmap);
    for i = 1:2:h
        for j = 1:w
            if (i ~= 1 && wallmap(i - 1, j) == 1)
                fprintf("|");
            else
                fprintf(" ");
            end 
            if mod(i, 2) == 1
                if (wallmap(i, j) == 1)
                    fprintf("_");
                else
                    fprintf(" ");
                end              
            end
                        
        end
            fprintf("\n")
    end
end

wallmap1 = top2walls(tmap1)
printwallmap(wallmap1)

wallmap2 = top2walls(tmap2)
printwallmap(wallmap2)

%BFS pathplanning
function nodelist = bfs(wallmap, originX, originY, targetx, targety)
    %Seed stuff
    w = width(wallmap) - 1;
    h = (height(wallmap) - 1)/2;
    currentX = originX;
    currentY = originY;
    cellstoExplore = [originX; originY; -999; -999];
    cellsExplored = [-999; -999; -999; -999];

    %Do the BFS
    while (width(cellstoExplore) > 0)
        currentX = cellstoExplore(1, 1);
        currentY = cellstoExplore(2, 1);
        cellsExplored = [cellsExplored(), cellstoExplore(:, 1)];
        cellstoExplore = cellstoExplore(:, 2:end);
        if wallmap(2*currentY -1, currentX) ~= 1%up
            if ~contains(cellsExplored, currentX, currentY - 1)
                cellstoExplore(:, end + 1) = [currentX, currentY - 1, currentX, currentY];
            end
        end
        if wallmap(2*currentY, currentX + 1) ~= 1 %right
            if ~contains(cellsExplored, currentX + 1, currentY)
                cellstoExplore(:, end + 1) = [currentX + 1, currentY, currentX, currentY];
            end
        end
        if wallmap(2*currentY + 1, currentX) ~= 1%down
            if ~contains(cellsExplored, currentX, currentY + 1)
                cellstoExplore(:, end + 1) = [currentX, currentY + 1, currentX, currentY];
            end
        end
        if wallmap(2*currentY, currentX) ~= 1 %left
            if ~contains(cellsExplored, currentX - 1, currentY)
                cellstoExplore(:, end + 1) = [currentX - 1, currentY, currentX, currentY];
            end
        end
        if contains(cellsExplored, targetx, targety)
            break;
        end
    end

    %Recreate path
    path = cellsExplored(1:2, end);
    while true
        path = [path, retraceRoute(cellsExplored, path(:, end))];
        if (path(1, end) == originX && path(2, end) == originY)
            break;
        end
    end
    nodelist = fliplr(path);
end

%Helper functions
function res = contains(cells, targetX, targetY)
    for i = 1:width(cells)
        if cells(1, i) == targetX && cells(2, i) == targetY
            res = 1;
            break;
        end
        res = 0;
    end
end

function previousCell = retraceRoute(cells, currentCell)
    for i = 1:width(cells)
        if cells(1:2, i) == currentCell
            previousCell = cells(3:4, i);
            break;
        end
    end
end

bfs(wallmap2, 3, 2, 1, 3)


%function [overlaps, origins] = localizeBasedOnMap(wallMap, currentMap)
%
%end

function newMap = rotateWallMap90(wallmap)
newMap = zeros((width(wallmap) - 1) * 2 + 1, (height(wallmap) - 1)/2 + 1);
%Populate odd rows

    for i = (1:2:height(newMap))
        for j = 1:width(newMap) - 1
            newMap(i, j) = wallmap(2*j, width(wallmap) + 1 - (i + 1) / 2);
        end
    end
%Populate even rows
    for i = 2:2:height(newMap)
        for j = 1:width(newMap)
            newMap(i, j) = wallmap(2*j - 1 ,i / 2);
        end
    end

end

rotatedmap = rotateWallMap90(wallmap2);
printwallmap(wallmap2)
printwallmap(rotatedmap)


%test wifi comms
fprintf("\nstarting comms\n");
%tcpclientfind("Port", 23)
client = tcpclient("192.168.137.60", 23, "Timeout", 5); %to check ip address: right click mobile hotspot, go to settings
fprintf("got client\n");

pause(1)
fprintf("sending a message\n");
client.write("Super grate message to send to the robot becasue sending messages is cool")
client.flush()
fprintf("done sending a message\n");


fprintf("Message received: '");
fprintf(client.readline())
fprintf("'\nDone receiving message\n");

%disconnect from robot
delete(client)