function U = compute_actions(actuators)
%COMPUTE_ACTIONS Summary of this function goes here
%   Detailed explanation goes here
    
    
    % Allocate space
    U = zeros(size(actuators,1), 2);

    U(:,1) = actuators(:,1) + -1 * actuators(:,2);
    U(:,2) = actuators(:,5);
end

