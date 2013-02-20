function S = compute_states(sensors, replaceNaN)
%COMPUTE_STATES Compute state representation from sensor information.
%   SENSORS is a matrix with each row containing full sensor information
%   for time t. 
    
    if nargin < 2
        replaceNaN = false;
    end

    % Allocate space
    S = zeros(size(sensors,1), 6);

    S(:,1) = sensors(:,47) ./ 3600 .* 1000;
    S(:,2) = sensors(:,48) ./ 3600 .* 1000;
    S(:,3) = find_yawrates_using_splines(sensors, replaceNaN);
    
    S(:,4:6) = sensors(:,[4 69 1]);
end

