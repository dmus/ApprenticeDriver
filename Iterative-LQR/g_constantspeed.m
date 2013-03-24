function cost = g_constantspeed(s)
%G_ACCELERATION Summary of this function goes here
%   Detailed explanation goes here
    
    % Penalize deviation from 5 km/h
    %cost = (s(1) - 1.3899) ^ 2;
    cost = (s(3) - 0.2)^2;
end

