function accelerations = compute_accelerations(trajectory)
%COMPUTE_ACCELERATIONS Summary of this function goes here
%   Detailed explanation goes here
    
    % Compute accelerations
    H = size(trajectory.S,1);
    Y = zeros(H-1, 3);
    for t = 1:H-1
        dt = trajectory.T(t+1) - trajectory.T(t);
        angle = trajectory.S(t+1,3) * dt;
        R = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
        Y(t,1:2) = ((R * trajectory.S(t+1,1:2)')' - trajectory.S(t,1:2)) / dt;
        Y(t,3) = (trajectory.S(t+1,3) - trajectory.S(t,3)) / dt;
    end

    accelerations = Y;
end

