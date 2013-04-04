function cost = h(u)
%H Cost for control input U.
%   The cost is determined by several control input features.

    % Limits should be respected
    a = max(0,abs(u)-1);
    cost = 10000 * (a' * a);
    cost = 0;

    %cost = (u(1) - 1)^2 + u(2)^2;
end

