classdef Client < handle
    %CLIENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Options
        port = 3001;                    % the port for the connection (default is 3001)
        host = 'localhost';             % the address of the host where the server is running (default is localhost)
        clientId = 'championship2010';  % the ID of the client sent to the server (default is championship2011)
        verbose = true;                 % to set verbose mode on (default is off)
        maxEpisodes = 1;                % to set the number of episodes (default is 1)
        maxSteps = 0;                   % the max number of steps for each episode (0 is default value, that means unlimited number of steps)
        stage = 3;                      % the current stage: 0 is WARMUP, 1 is QUALIFYING, 2 is RACE, others value means UNKNOWN (default is UNKNOWN)
        trackName = 'unknown';          % the name of the current track will be passed to the driver
        driver;                         % Call the constructor of the selected driver class to create a driver object

        % Do not change this options
        UDP_TIMEOUT = 10000;
    end
    
    methods
        function obj = Client(driver, maxSteps, maxEpisodes)
            obj.driver = driver;
            obj.maxSteps = maxSteps;
            obj.maxEpisodes = maxEpisodes;
        end
        
        function run(this)
            warning('off', 'all');
            javarmpath('./SCR-Client');
            javaaddpath('./SCR-Client');
            warning('on', 'all');
            socket = SocketHandler(this.host, this.port, this.verbose);

            this.driver.setStage(this.stage);
            this.driver.setTrackName(this.trackName);

            angles = this.driver.initAngles();
            initStr = strcat(this.clientId, '(init');
            for i = 1:length(angles)
                initStr = strcat(initStr, {' '}, int2str(angles(i)));
            end
            initStr = strcat(initStr, ')');

            curEpisode = 1;
            shutdownOccured = false;

            % Control loop
            while ~shutdownOccured && curEpisode <= this.maxEpisodes
                % First client identification
                inMsg = '';
                while strcmp(inMsg, '') || isempty(strfind(inMsg, '***identified***'))
                    socket.send(initStr);
                    inMsg = char(socket.receive(this.UDP_TIMEOUT));
                end
                disp('Client identified');
                % Start driving
                curStep = 0;
                while true
                    % Receive game state from TORCS
                    inMsg = char(socket.receive(this.UDP_TIMEOUT));
                    if ~strcmp(inMsg, '')
                        % Check if race is ended (shutdown)
                        if ~isempty(strfind(inMsg, '***shutdown***'))
                            shutdownOccurred = true;
                            disp('Server shutdown!');
                            break;
                        end

                        % Check if race is restarted
                        if ~isempty(strfind(inMsg, '***restart***'))
                            this.driver.reset();
                            if this.verbose
                                disp('Server restarting!');
                            end
                            break;
                        end

                        % Default values
                        action = zeros(7,1);
                        action(6) = false;
                        action(7) = 360;
                        if (curStep <= this.maxSteps || this.maxSteps == 0)
                            sensors = sscanf(inMsg, '(angle %f)(curLapTime %f)(damage %f)(distFromStart %f)(distRaced %f)(fuel %f)(gear %f)(lastLapTime %f)(opponents %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(racePos %f)(rpm %f)(speedX %f)(speedY %f)(speedZ %f)(track %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(trackPos %f)(wheelSpinVel %f %f %f %f)(z %f)(focus %f %f %f %f %f)');
                            
                            if sensors(2) >= 0
                                curStep = curStep + 1;
                            end
                            
                            action = this.driver.control(sensors);
                        else
                            action(6) = true; % Restart race
                        end

                        % Limit values
                        minValues = [0; 0; 0; -1; -1];
                        maxValues = [1; 1; 1; 6; 1];
                        action(1:5) = max(min(action(1:5), maxValues), minValues);

                        actionStr = sprintf('(accel %f) (brake %f) (clutch %f) (gear %f) (steer %f) (meta %d) (focus %d)', action);
                        socket.send(actionStr);
                    else
                        disp('Server did not respond within the timeout');
                    end
                end

                curEpisode = curEpisode + 1;
            end

            % Shutdown
            this.driver.shutdown();
            socket.close();
            disp('Client shutdown.');
            disp('Bye, bye!');
        end
    end
    
end

