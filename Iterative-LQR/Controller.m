classdef Controller < handle
    %CONTROLLER Controller for a car in the SCR championships.
    %   Controller based on iterative LQR algorithm.
    
    properties
        t                   % Current timestep
        sensors             % Current raw sensor readings
        sensorsPrev        % Sensor readings at time t-1
        state               % Current full state
        K                   % Optimal time-varying policy
        P                   % Time-varying cost-to-go function
        model               % Acceleration-model for prediction of next state
        H                   % Horizon
        history             % Previous states and actions for this trial
        reference           % Reference trajectory
        map                 % Map representing the track
        alpha               % Weight parameter for penalizing deviations
        episode             % Number of current trial
        rangeFinderIndices  % Sensor elements corresponding to range finders
        angles              % Range finder configuration
        options             % Options for fminsearch
        A                   % LQR dynamics matrix
        B                   % LQR dynamics matrix
        Q                   % LQR cost matrix
        R                   % LQR cost matrix
        vPrev
        stateDim           % Number of elements in state
        actionDim          % Number of elements in action
        bias
        useBias            % Setting for using inaccurate model algoritm or not
    end
    
    methods
        function obj = Controller(H, reference, model, map)
            disp('Initializing controller...');
            
            % Horizon
            obj.H = H;
            
            obj.reference = reference;
            
            obj.stateDim = size(obj.reference.S,2);
            obj.actionDim = size(obj.reference.U,2);
            
            obj.map = map;
            obj.model = model;
            
            % Initialization
            obj.episode = 0;
            %obj.previousState = zeros(stateLength,1);
            
            obj.alpha = 0; 
            obj.rangeFinderIndices = (1:19) + 49;
            obj.angles = transp(((obj.rangeFinderIndices - 10) / 9) * (0.5*pi) * -1);
            obj.options = optimset('LargeScale', 'off', 'TolX', 0.0001, 'Display', 'off');
            obj.useBias = false;
            
            obj.onStart();
            
            disp('Controller ready...');
        end
        
        % Initialize state of driver for a new episode
        function onStart(this)
            this.t = 0;
            this.episode = this.episode + 1;
            this.vPrev = zeros(this.actionDim,1);
            this.state = zeros(this.stateDim,1);
            
            this.history.S = zeros(this.H, this.stateDim);
            this.history.U = zeros(this.H, this.actionDim);
            this.history.T = zeros(this.H, 1);
            
            % Compute first time-varying policy
            if this.useBias
                % Compute bias for time-dependent dynamics function
                this.bias = zeros(this.H-1, this.stateDim);
                for t = 1:this.H - 1
                    dt = this.reference.T(t+1) - this.reference.T(t);
                    prediction = f(this.reference.S(t,:)', this.reference.U(t,:)', dt, this.map, this.model);
                    this.bias(t,:) = this.reference.S(t+1,:) - prediction';
                end
                
            else
                this.bias = zeros(this.H-1, this.stateDim);
            end
                
            [this.A, this.B, this.Q, this.R] = approximate_lqr(this.reference, this.alpha, this.model, this.map, this.bias);
            [this.K, this.P] = create_time_varying_controller(this.A, this.B, this.Q, this.R, this.Q{this.H});
        end
        
        % Sensors is a long vector containing sensor measurements
        function action = control(this, sensors)
            % Save previous sensor readings
            this.sensorsPrev = this.sensors;
            this.sensors = sensors;
            
            % If start sign is not given yet
            if this.sensors(2) < 0
                this.state = [this.sensors([47 48]); 0];%; this.sensors([4 69 1]); 1; this.previousState(1:7)];%; this.previousAction];
                u = zeros(this.action_dim,1);
                return;
            end
            
            % Compute current state and compute control action
            s = this.computeState();
            u = this.controlFromState(s, this.sensors(2)); % TODO fix for new lap
            
            % Check if accelerating or braking
            if u(1) > 0
                action = [u(1); 0; u(2)];
            else
                action = [0; -1 * u(1); u(2)];
            end
        end
        
        function u = controlFromState(this, state, curtime)
            % Increase timestep
            this.t = this.t + 1;
            
            % Save previous state
            %this.statePrev = this.state;
            this.state = state;
            
            this.history.T(this.t,:) = curtime;
            this.history.S(this.t,:) = this.state';
            
            if this.t >= this.H
                % Output does not matter, because trial has ended
                u = zeros(this.action_dim,1);
                return;
            end
            
            % Compute change in control inputs
            z = [this.state - this.reference.S(this.t,:)'; 1; this.vPrev; 1];
            dv = this.K{this.t} * z;
            v = this.vPrev + dv;
            u = v + this.reference.U(this.t,:)';
            this.history.U(this.t,:) = u;
            
            % For next timestep
            this.vPrev = v;
        end
        
        function reset(this)
            disp('resetting...');
            
            % Compute total costs
            cost = 0;
            for t = 1:this.H - 1
                cost = cost + g(this.history.S(t,:)') + h(this.history.U(t,:)');
            end
            cost = cost + g(this.history.S(this.H,:)');
            
            fprintf('Final cost for episode %i: %f\n', this.episode, cost);
            
            this.alpha = this.alpha - 0.1;
            
            % Compute and solve new optimal control problem
            
            % this.reference = this.history;
            
            this.onStart();
        end
        
        function shutdown(this)
            disp('Shutdown...');
        end
        
        function s = computeState(this)
            s = zeros(size(this.state));
            
            % Forward and lateral speed in m/s
            s(1:2) = this.sensors(47:48) * 1000 / 3600;
            %s(1) = this.sensors(47) * 1000 / 3600;
            
            
            % Angular velocity in rad/s
            s(3) = this.estimateYawrate();

%             % State features
%             s(4:6) = this.sensors([4 69 1]);
%             
%             % And remaining elements
%             s(7) = 1;
%             s(8:14) = this.previousState(1:7);
%             
%             % Also previous action in state
%             %s(15:17) = this.previousAction;
        end
        
        % Estimate yaw rate at time t.
        function yawrate = estimateYawrate(this)
            % TODO fix dt computation for more than 1 full lap
            dt = this.sensors(2) - this.sensorsPrev(2);
            
            % Compensation parameter 
            ALPHA = 0.805; 
            
            % Current guess
            yawRate = 0;%this.previousState(3);
            
            % Compute marks at time t-1 for the reference scan
            ranges = this.sensorsPrev(this.rangeFinderIndices);
            marks = [ranges .* cos(this.angles) ranges .* sin(this.angles)];
            
            try
                [marksLeft, marksRight] = groupRangeFinders(marks, 10);

                % Compute move in body coordinate frame
                move = (this.statePrev(1:2)' + this.state(1:2)') .* 0.5 .* dt;

                % Compute marks at time t for the current scan
                newRanges = this.sensors(this.rangeFinderIndices);
                newMarks = [newRanges .* cos(this.angles) newRanges .* sin(this.angles)];
                [newMarksLeft, newMarksRight] = groupRangeFinders(newMarks, 10);

                yawrate = fminsearch(@(x) computeProjectionError(marksLeft, marksRight, move, newMarksLeft, newMarksRight, x), yawRate, this.options);

                yawrate = yawRate * ALPHA;
                
                % Now we have a better estimate for the yawrate at time t-1
                yawrate = yawrate / dt;
            catch e
                disp('Error in yaw rate estimation');
            end
        end
        
    end
end
