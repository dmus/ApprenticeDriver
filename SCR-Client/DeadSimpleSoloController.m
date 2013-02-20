classdef DeadSimpleSoloController < Driver
%DEADSIMPLESOLOCONTROLLER Summary of this class goes here
%   Detailed explanation goes here

   properties
       targetSpeed = 10;
   end

   methods
       function action = control(this, sensors)
           action = zeros(7,1);
           action(6) = false;
           action(7) = 360;
           
           if sensors(47) < this.targetSpeed
               action(1) = 1;
           end

           if sensors(1) < 0
               action(5) = -0.1;
           else
               action(5) = 0.1;
           end

           action(4) = 1;
       end
       
       function reset(this)
           disp('Resetting the race!');
       end
       
       function shutdown(this)
           disp('Bye Bye!');
       end
   end
end 
