function trajectory = build_trajectory(filename, k, H)
%BUILD_TRAJECTORY Build a state action trajectory from a log file.
%   FILENAME is the log file, K the total number of states in a state and H
%   the horizon.
    S = load(filename);
    
    % Remove states and actions before start signal
    sensors = S.States(S.States(:,2) > 0,:);
    actuators = S.Actions(S.States(:,2) > 0,:);
    
    H = min(H,size(sensors,1));
    
    times = compute_discretized_times(sensors(1:H,:));
    
    X = zeros(H, k*8);
    
    X(:,3:8) = compute_states(sensors(1:H,:), true);
    U = compute_actions(actuators(1:H,:));
    X(:,1:2) = U;
    
    % Rotate
    for i = 0:k-2
        % 2 control inputs, 6 state inputs, total 8
        first = i*8+1;
        last = first+8-1;
        
        U_prev = X(:,first:first+2-1);
        S_prev = X(:,first+2:last);
        
        for t = 1:H-(i+1)
            dt = times(t+1) - times(t);
            angle = S_prev(t+1,3) * dt;
            Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            S_prev(t,1:2) = (Rot * S_prev(t,1:2)')';
        end
        
        X(2:end,first+8:last+8) = [U_prev(1:end-1,:) S_prev(1:end-1,:)];
    end
    
    S = X(:,3:end);
    
    trajectory.S = S(1:H,:);
    trajectory.U = U;
    trajectory.T = times;
end

