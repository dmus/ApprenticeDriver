function cost = g_constantspeed(s)
%G_ACCELERATION Summary of this function goes here
%   Detailed explanation goes here
    
    % Penalize deviation from 5 km/h
    cost = (s(1) - 13.8889) ^2 + s(3) ^2;
end

