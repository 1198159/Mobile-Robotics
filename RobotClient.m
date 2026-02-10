classdef RobotClient
    properties
        client
        isConnected
        currentX
        currentY
        positionError
    end
    
    methods
        function obj = RobotClient()
            obj.isConnected = false;
            obj.client = [];
            obj.currentX = 1;
            obj.currentY = 1;
            obj.positionError = 0;
        end
            
        function [obj, response] = connect(obj)
            % Find robot IP address
            [status, result] = system('arp -a');
            pat = regexpPattern("192\.168\.137\.245");
            ipAddrArray = extract(result, pat);
            
            [r, c] = size(ipAddrArray);
            
            if r == 1
                ipAddr = ipAddrArray{1};
                fprintf("Found IP: %s\n", ipAddr);
                
                % Connect as TCP client
                obj.client = tcpclient(ipAddr, 23, "Timeout", 5);
                obj.isConnected = true;
                fprintf("Connected to robot\n");
                
                % Send connection handshake
                response = obj.sendMessage("CONNECT");
            else
                fprintf("No IP found, check if hotspot is on and robot is on\n");
                response = "Connection failed: No IP found";
                obj.isConnected = false;
            end
        end
        
        function response = sendMessage(obj, message)
            if ~obj.isConnected || isempty(obj.client)
                response = "Not connected";
                return;
            end
            
            try
                obj.client.write(message);
                obj.client.flush();
                response = obj.client.readline();
            catch e
                response = sprintf("Error: %s", e.message);
            end
        end
        
        function response = pollSensors(obj)
            response = obj.sendMessage("POLL_SENSORS");
        end
        
        function [obj, success, statusMsg] = moveToPosition(obj, targetX, targetY)
            % Send absolute target coordinates to robot
            % Protocol: "MOVETO,X,Y" -> Response: "POS,currentX,currentY,error"
            
            command = sprintf("MOVETO,%d,%d", targetX, targetY);
            response = obj.sendMessage(command);
            
            % Parse response
            [obj, success, statusMsg] = obj.parsePositionResponse(response);
        end
        
        function [obj, success, statusMsg] = getCurrentPosition(obj)
            % Request current position from robot
            % Protocol: "GETPOS" -> Response: "POS,currentX,currentY,error"
            
            response = obj.sendMessage("GETPOS");
            [obj, success, statusMsg] = obj.parsePositionResponse(response);
        end
        
        function [obj, success, statusMsg] = parsePositionResponse(obj, response)
            % Parse robot response: "POS,X,Y,error"
            % Returns success flag and status message
            
            success = false;
            statusMsg = response;
            
            try
                % Split response by comma
                parts = strsplit(char(response), ',');
                
                if length(parts) >= 4 && strcmp(parts{1}, 'POS')
                    % Extract position data
                    obj.currentX = str2double(parts{2});
                    obj.currentY = str2double(parts{3});
                    obj.positionError = str2double(parts{4});
                    
                    success = true;
                    statusMsg = sprintf("Position: (%d, %d), Error: %.2f", ...
                        obj.currentX, obj.currentY, obj.positionError);
                    
                    fprintf("%s\n", statusMsg);
                elseif strcmp(parts{1}, 'ERROR')
                    % Robot sent error message
                    if length(parts) >= 2
                        statusMsg = sprintf("Robot Error: %s", strjoin(parts(2:end), ','));
                    else
                        statusMsg = "Robot Error: Unknown error";
                    end
                    fprintf("%s\n", statusMsg);
                else
                    % Unexpected response format
                    statusMsg = sprintf("Unexpected response: %s", response);
                    fprintf("%s\n", statusMsg);
                end
            catch e
                statusMsg = sprintf("Parse error: %s", e.message);
                fprintf("%s\n", statusMsg);
            end
        end
        
        function [obj, success, statusMsg] = followPath(obj, path)
            % Send robot through a series of waypoints
            % path is a 2xN matrix where row 1 is X coords, row 2 is Y coords
            
            success = true;
            statusMsg = "";
            
            if size(path, 2) < 1
                success = false;
                statusMsg = "Empty path";
                return;
            end
            
            % Move through each waypoint in the path
            for i = 1:size(path, 2)
                targetX = path(1, i);
                targetY = path(2, i);
                
                fprintf("Moving to waypoint %d: (%d, %d)\n", i, targetX, targetY);
                [obj, waypoint_success, waypoint_msg] = obj.moveToPosition(targetX, targetY);
                
                if ~waypoint_success
                    success = false;
                    statusMsg = sprintf("Failed at waypoint %d: %s", i, waypoint_msg);
                    return;
                end
                
                % Optional: Add delay between waypoints if needed
                % pause(0.5);
            end
            
            statusMsg = sprintf("Path completed. Final position: (%d, %d), Error: %.2f", ...
                obj.currentX, obj.currentY, obj.positionError);
        end
        
        function [obj, success, statusMsg] = setPosition(obj, x, y)
            % Manually set robot's believed position (for calibration)
            % Protocol: "SETPOS,X,Y" -> Response: "POS,X,Y,0"
            
            command = sprintf("SETPOS,%d,%d", x, y);
            response = obj.sendMessage(command);
            [obj, success, statusMsg] = obj.parsePositionResponse(response);
        end
        
        function errorValue = getPositionError(obj)
            % Return the last known position error
            errorValue = obj.positionError;
        end
        
        function disconnect(obj)
            if obj.isConnected && ~isempty(obj.client)
                % Send disconnect message
                try
                    obj.sendMessage("DISCONNECT");
                catch
                    % Ignore errors during disconnect
                end
                
                delete(obj.client);
                obj.isConnected = false;
                fprintf("Disconnected from robot\n");
            end
        end
    end
end