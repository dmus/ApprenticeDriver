classdef ClutchBehaviour < handle
    %CLUTCHBEHAVIOUR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        clutch = 0;
        
        clutchMax = 0.5;
        clutchDeltaTime = 0.02;
        clutchDeltaRaced = 10;
        clutchDelta = 0.05;
        clutchMaxModifier = 1.3;
        clutchMaxTime = 1.5;
        clutchDec = 0.01;
    end
    
    methods
        function action = execute(this, sensors, action)
            maxClutch = this.clutchMax;

            % Check if the current situation is the race start
            if sensors(2) < this.clutchDeltaTime && sensors(5) < this.clutchDeltaRaced
                this.clutch = maxClutch;
            end

            % Adjust the current value of the clutch
            if this.clutch > 0
                delta = this.clutchDelta;
            
                if sensors(4) < 2
                    % Apply a stronger clutch output when the gear is one and the race is just started
                    delta = delta / 2;
                    maxClutch = maxClutch * this.clutchMaxModifier;
                    if sensors(2) < this.clutchMaxTime
                        this.clutch = maxClutch;
                    end
                end

                % check clutch is not bigger than maximum values
                this.clutch = min(maxClutch, this.clutch);

                % if clutch is not at max value decrease it quite quickly
                if this.clutch ~= maxClutch
                    this.clutch = this.clutch - delta;
                    this.clutch = max(0.0, this.clutch);
                else
                    % if clutch is at max value decrease it very slowly
                    this.clutch = this.clutch - this.clutchDec;
                end
            end
            
            action(3) = this.clutch;
        end
        
        function reset(this)
            this.clutch = 0;
        end
    end
    
end

