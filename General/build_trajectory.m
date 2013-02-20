function trajectory = build_trajectory(filename, k, H)
%BUILD_TRAJECTORY Build a state action trajectory from a log file.
%   FILENAME is the log file, K the total number of states in a state and H
%   the horizon.
    S = load(filename);
    
    % Remove states and actions before start signal
    sensors = S.States(S.States(:,2) > 0,:);
    actuators = S.Actions(S.States(:,2) > 0,:);

    
    S = zeros(H, k*8-2);
    
    S(:,1:6) = compute_states(sensors(1:H,:), true);
    U = compute_actions(actuators(1:H-1,:));
    
    % TODO rotate to right body frame
    j = 6; % Pointer to last element that is speficied
    for i = 1:k-1
        S(1+i:end,j+1:j+2) = U(1:end-i+1,:);
        j = j+2;
        S(1:1+i-1,j+4:j+6) = repmat(S(1,4:6),1+i-1,1);
        S(1+i:end,j+1:j+6) = S(1:end-i,1:6);
        j = j+6;
    end
    
    trajectory.S = S(1:H,:);
    trajectory.U = U;
    trajectory.T = compute_discretized_times(sensors);
end

