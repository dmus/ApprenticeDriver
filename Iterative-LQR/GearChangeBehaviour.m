classdef GearChangeBehaviour < handle
    %GEARCHANGEBEHAVIOUR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        counter = 0;
        WAIT_TIME = 1 * 50;
        gear = 1;   
        UPSHIFT = 9200.0;
        DOWNSHIFT_2ND = 3500.0;
        DOWNSHIFT = 6000.0;  
    end
    
    methods
        function action = execute(this, sensors, action)
            if sensors(7) ~= this.gear
                % do nothing
            elseif sensors(46) > this.UPSHIFT && sensors(7) < 6 && this.counter < 1
                this.gear = sensors(7) + 1;
                this.counter = this.WAIT_TIME;
            elseif sensors(46) < this.DOWNSHIFT_2ND && sensors(7) == 2 && this.counter < 1
                this.gear = sensors(7) - 1;
                this.counter = this.WAIT_TIME;
            elseif sensors(46) < this.DOWNSHIFT && sensors(7) > 2 && this.counter < 1
                this.gear = sensors(7) - 1;
                this.counter = this.WAIT_TIME;
            elseif sensors(7) < 1
                this.gear = 1;
                this.counter = this.WAIT_TIME;
            else
                this.counter = this.counter - 1;
            end
                
            action(4) = this.gear;
            action(3) = 0.0;
        end
        
        function reset(this)
            this.counter = 0;
            this.gear = 1;
        end
    end
    
end

