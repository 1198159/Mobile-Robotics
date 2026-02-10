classdef PathPlanner
    methods (Static)
        function nodelist = bfs(wallmap, originX, originY, targetx, targety)
            %Seed stuff
            w = width(wallmap) - 1;
            h = (height(wallmap) - 1)/2;
            currentX = originX;
            currentY = originY;
            cellstoExplore = [originX; originY; -999; -999];
            cellsExplored = [-999; -999; -999; -999];
            
            if originX == targetx && originY == targety
                nodelist = [originX; originY];
                return;
            end
            
            %Do the BFS
            while (width(cellstoExplore) > 0)
                currentX = cellstoExplore(1, 1);
                currentY = cellstoExplore(2, 1);
                cellsExplored = [cellsExplored, cellstoExplore(:, 1)];
                cellstoExplore = cellstoExplore(:, 2:end);
                
                if wallmap(2*currentY -1, currentX) ~= 1 %up
                    if ~PathPlanner.contains(cellsExplored, currentX, currentY - 1)
                        cellstoExplore(:, end + 1) = [currentX; currentY - 1; currentX; currentY];
                    end
                end
                if wallmap(2*currentY, currentX + 1) ~= 1 %right
                    if ~PathPlanner.contains(cellsExplored, currentX + 1, currentY)
                        cellstoExplore(:, end + 1) = [currentX + 1; currentY; currentX; currentY];
                    end
                end
                if wallmap(2*currentY + 1, currentX) ~= 1 %down
                    if ~PathPlanner.contains(cellsExplored, currentX, currentY + 1)
                        cellstoExplore(:, end + 1) = [currentX; currentY + 1; currentX; currentY];
                    end
                end
                if wallmap(2*currentY, currentX) ~= 1 %left
                    if ~PathPlanner.contains(cellsExplored, currentX - 1, currentY)
                        cellstoExplore(:, end + 1) = [currentX - 1; currentY; currentX; currentY];
                    end
                end
                
                if PathPlanner.contains(cellsExplored, targetx, targety)
                    break;
                end
            end

            %Recreate path
            path = cellsExplored(1:2, end);
            while true
                path = [path, PathPlanner.retraceRoute(cellsExplored, path(:, end))];
                if (path(1, end) == originX && path(2, end) == originY)
                    break;
                end
            end
            nodelist = fliplr(path);
        end
        
        function res = contains(cells, targetX, targetY)
            res = false;
            for i = 1:size(cells,2)
                if isequal(cells(1:2,i), [targetX; targetY])
                    res = true;
                    break
                end
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
    end
end