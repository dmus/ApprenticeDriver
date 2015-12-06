function cost = g_constantspeed(s)
%G_ACCELERATION Summary of this function goes here
%   Detailed explanation goes here
    
    % Penalize deviation from 50 km/h
    cost = (s(1) - 13.8889)^2 + 100*s(3)^2;
end

