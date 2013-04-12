classdef Controller < handle
    %CONTROLLER Controller for a car in the SCR championships.
    %   Controller based on iterative LQR algorithm.
    
    properties
        t                   % Current timestep
        sensors             % Current raw sensor readings
        sensorsPrev         % Sensor readings at time t-1
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
        stateDim            % Number of elements in state
        actionDim           % Number of elements in action
        bias
        useBias             % Setting for using inaccurate model algoritm or not
        stage
        trackName
        u
        gearChangeBehaviour
        clutchBehaviour
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
            indices = 1:19;
            obj.angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);
            obj.options = optimset('LargeScale', 'off', 'TolX', 0.0001, 'Display', 'off');
            obj.useBias = false;
            
            obj.gearChangeBehaviour = GearChangeBehaviour();
            obj.clutchBehaviour = ClutchBehaviour();
            
            obj.onStart();
            
            disp('Controller ready...');
        end
        
        % Initialize state of driver for a new episode
        function onStart(this)
            this.t = 0;
            this.episode = this.episode + 1;
            this.vPrev = zeros(this.actionDim,1);
            this.state = zeros(this.stateDim,1);
            this.u = zeros(this.actionDim,1);
            
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
            tic
            [this.A, this.B, this.Q, this.R] = approximate_lqr(this.reference, @f, @g_constantspeed, this.alpha, this.model, this.map, this.bias);
            toc
            [this.K, this.P] = create_time_varying_controller(this.A, this.B, this.Q, this.R, this.Q{this.H});
        end
        
        function setStage(this, stage)
            this.stage = stage;
        end
        
        function setTrackName(this, trackName)
            this.trackName = trackName;
        end
        
        function angles = initAngles(this)
            angles = -90:10:90;
        end
        
        % Sensors is a long vector containing sensor measurements
        function action = control(this, sensors)
            % Save previous sensor readings
            this.sensorsPrev = this.sensors;
            this.sensors = sensors;
            
            action = zeros(7,1);
            
            % If start sign is not given yet
            if this.sensors(2) < 0
                return;
            end
            
            % Compute current state and compute control action
            s = this.computeState();
            u = this.controlFromState(s, this.sensors(2)); % TODO fix for new lap
            
            % Store u
            this.u = u;
            
            % Check if accelerating or braking
            if u(1) > 0
                action([1 2 5]) = [u(1); 0; u(2)];
            else
                action([1 2 5]) = [0; -1 * u(1); u(2)];
            end
            
            action = this.gearChangeBehaviour.execute(sensors, action);
            action = this.clutchBehaviour.execute(sensors, action);
        end
        
        function u = controlFromState(this, state, curtime)
            % Increase timestep
            this.t = this.t + 1;
            
            % Save state
            this.state = state;
            
            this.history.T(this.t,:) = curtime;
            this.history.S(this.t,:) = this.state';
            
            if this.t >= this.H
                % Output does not matter, because trial has ended
                u = zeros(this.actionDim,1);
                return;
            end
            
            % Compute change in control inputs
            z = [this.state - this.reference.S(this.t,:)'; 1; this.vPrev; 1];
            dv = this.K{this.t} * z;
            v = this.vPrev + dv;
            u = v + this.reference.U(this.t,:)';
            
            % Cap u
            u(u > 1) = 1;
            u(u < -1) = -1;
            
            this.history.U(this.t,:) = u;
            
            % DEBUG BEGIN
%             fun = @(x) z' * this.Q{this.t} * z + x' * this.R{this.t} * x + (this.A{this.t} * z + this.B{this.t} * x)' * this.P{this.t+1} * (this.A{this.t} * z + this.B{this.t} * x);
%             vprime = fminsearch(fun,[0 0]');
%             
%             cost1 = z' * this.Q{this.t} * z;
%             cost2 = g_constantspeed(this.state) + h(u);
%             
%             s_plus1 = f(this.state,u,.02,this.model,this.map);
%             z_plus1 = this.A{this.t} * z + this.B{this.t} * dv;
%             s_plus1L = z_plus1(1:78) + this.reference.S(this.t+1,1:78)';
%             
%             v_plus1 = u - this.reference.U(this.t+1,:)';
%             dv_plus1 = v_plus1 - v;
%             s_plus2 = f(s_plus1, u, .02, this.model, this.map);
%             z_plus2 = this.A{this.t+1} * z_plus1 + this.B{this.t+1} * dv_plus1;
%             s_plus2L = z_plus2(1:78) + this.reference.S(this.t+2,1:78)';
%             
%             v_plus2 = u - this.reference.U(this.t+2,:)';
%             dv_plus2 = v_plus2 - v_plus1;
%             s_plus3 = f(s_plus2, u, .02, this.model, this.map);
%             z_plus3 = this.A{this.t+2} * z_plus2 + this.B{this.t+2} * dv_plus2;
%             s_plus3L = z_plus3(1:78) + this.reference.S(this.t+3,1:78)';
%             
%             v_plus3 = u - this.reference.U(this.t+3,:)';
%             dv_plus3 = v_plus3 - v_plus2;
%             s_plus4 = f(s_plus3, u, .02, this.model, this.map);
%             z_plus4 = this.A{this.t+3} * z_plus3 + this.B{this.t+3} * dv_plus3;
%             s_plus4L = z_plus4(1:78) + this.reference.S(this.t+4,1:78)';
%             
%             % Also look at an alternative action
%             u_ = [1;-1];
%             v_ = u_ - this.reference.U(this.t,:)';
%             dv_ = v_ - this.vPrev;
%             z_plus1_ = this.A{this.t} * z + this.B{this.t} * dv_;
%             
%             s_plus1_ = f(this.state,u_,.02,this.model,this.map);
%             s_plus1L_ = z_plus1_(1:78) + this.reference.S(this.t+1,1:78)';
%             
%             v_plus1_ = u_ - this.reference.U(this.t+1,:)';
%             dv_plus1_ = v_plus1_ - v_;
%             s_plus2_ = f(s_plus1_, u_, .02, this.model, this.map);
%             z_plus2_ = this.A{this.t+1} * z_plus1_ + this.B{this.t+1} * dv_plus1_;
%             s_plus2L_ = z_plus2_(1:78) + this.reference.S(this.t+2,1:78)';
%             
%             v_plus2_ = u_ - this.reference.U(this.t+2,:)';
%             dv_plus2_ = v_plus2_ - v_plus1_;
%             s_plus3_ = f(s_plus2_, u_, .02, this.model, this.map);
%             z_plus3_ = this.A{this.t+2} * z_plus2_ + this.B{this.t+2} * dv_plus2_;
%             s_plus3L_ = z_plus3_(1:78) + this.reference.S(this.t+3,1:78)';
%             
%             v_plus3_ = u_ - this.reference.U(this.t+3,:)';
%             dv_plus3_ = v_plus3_ - v_plus2_;
%             s_plus4_ = f(s_plus3_, u_, .02, this.model, this.map);
%             z_plus4_ = this.A{this.t+3} * z_plus3_ + this.B{this.t+3} * dv_plus3_;
%             s_plus4L_ = z_plus4_(1:78) + this.reference.S(this.t+4,1:78)';
%             
            % DEBUG END
            
            % For next timestep
            this.vPrev = v;
        end
        
        function reset(this)
            disp('resetting...');
            
            this.clutchBehaviour.reset();
            this.gearChangeBehaviour.reset();
            %this.alpha = this.alpha - 0.1;
            
            % Compute and solve new optimal control problem
            
            this.reference = this.history;
            
            this.onStart();
        end
        
        function shutdown(this)
            disp('Shutdown...');
        end
        
        function s = computeState(this)
            s = zeros(size(this.state));
            
            % Forward and lateral speed in m/s
            s(1:2) = this.sensors(47:48) * 1000 / 3600;
            
            % Angular velocity in rad/s
            if abs(this.sensors(69)) <= 1
                s(3) = this.estimateYawrate();
            else
                s(3) = NaN;
            end

            % State features
            s(4:6) = this.sensors([4 69 1]);
            
            % Control input from previous timestep
            s(7:8) = this.u;
            
            % Shift previous states
            s(9:end) = this.state(1:end-8);
        end
        
        % Estimate yaw rate at time t.
        function yawrate = estimateYawrate(this)
            % TODO fix dt computation for more than 1 full lap
            dt = this.sensors(2) - this.sensorsPrev(2);
            
            % Compensation parameter 
            ALPHA = 0.805; 
            
            % Current guess
            yawrate = 0;%this.previousState(3);
            
            % Compute marks at time t-1 for the reference scan
            ranges = this.sensorsPrev(this.rangeFinderIndices);
            marks = [ranges .* cos(this.angles) ranges .* sin(this.angles)];
            
            try
                [marksLeft, marksRight] = groupRangeFinders(marks, 10);

                % Compute move in body coordinate frame
                move = (this.state(9:10)' + this.state(1:2)') .* 0.5 .* dt;

                % Compute marks at time t for the current scan
                newRanges = this.sensors(this.rangeFinderIndices);
                newMarks = [newRanges .* cos(this.angles) newRanges .* sin(this.angles)];
                [newMarksLeft, newMarksRight] = groupRangeFinders(newMarks, 10);

                yawrate = fminsearch(@(x) computeProjectionError(marksLeft, marksRight, move, newMarksLeft, newMarksRight, x), yawrate, this.options);

                yawrate = yawrate * ALPHA;
                
                yawrate = yawrate / dt;
            catch e
                %error('Error in yaw rate estimation');
                reference = this.sensorsPrev;
                current = this.sensors;
                history = this.history;
                save('debug.mat','reference','current','history');
                rethrow(e);
            end
        end
        
    end
end
