classdef PidController
%PIDCONTROLLER Summary of this class goes here
%   Detailed explanation goes here

   properties
       targetSpeed = 20;
   end

   methods
       function action = control(this, sensors)
           CTE = sensors(69);
           
       end
       
       function reset(this)
       end
       
       function shutdown(this)
       end
   end
end 
