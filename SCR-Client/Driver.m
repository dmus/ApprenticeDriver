classdef Driver < handle
%APPRENTICEDRIVER. Summary of this class goes here
%   Detailed explanation goes here

   properties
       WARM_UP = 0;
       QUALIFYING = 1;
       RACE = 2;
       UNKNOWN = 3;
       stage
       trackName
   end

   methods
       function obj = Driver()
           
       end
       
       function setStage(this, stage)
           this.stage = stage;
       end
       
       function setTrackName(this, name)
           this.trackName = name;
       end
       
       function angles = initAngles(this)
           	angles = -90:10:90;
       end
   end

   methods (Abstract)
       control(this, sensors);
       reset(this);
       shutdown(this);
   end
end 
