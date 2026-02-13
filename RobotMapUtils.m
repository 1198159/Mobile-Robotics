classdef RobotMapUtils
    methods (Static)
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

        function output = printwallmap(wallmap, xpos, ypos)
            w = width(wallmap);
            h = height(wallmap);
            output = '';
            for i = 1:2:h
                for j = 1:w
                    if (i ~= 1 && wallmap(i - 1, j) == 1)
                        output = output + "|";
                    else
                        output = output + " ";
                    end
                    if mod(i, 2) == 1
                        if (wallmap(i, j) == 1)
                            if (ypos == (i - 1)/2 && xpos == j)
                                output = output + "Δ";
                            else
                                output = output + "_";
                            end
                        else
                            if (ypos == (i - 1)/2 && xpos == j)
                                output = output + "^";
                            else
                                output = output + " ";
                            end
                        end
                    end
                end
                output = output + newline;
            end
        end
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
        function newMap = newMap()
            newMap = [2 0; 2 2; 2 0;]
        end
    end
end 