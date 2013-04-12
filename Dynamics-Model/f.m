function s_next = f(s, u, dt, model, map)
%F Predicts next state.
%   Accelerations are predicted and MAP is used to determine new position
%   and orientation.
    
    % Make sure limits are respected in control inputs
    u(u > 1) = 1;
    u(u < -1) = -1;

    s_next = zeros(size(s));
    k = (length(s) + 2) / 8;
    x = [u; s];
    for i = 0:k-2
        first = i*8+1;
        last = first+8-1;
        
        u_prev = x(first:first+2-1);
        s_prev = x(first+2:last);
        
        % Rotate previous velocities to new body frame
        angle = s(3) * dt;
        Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
        s_prev(1:2) = Rot * s_prev(1:2);
        
        % Shift previous input and state
        s_next(first+8-2:last+8-2) = [u_prev; s_prev];
    end   
     
    x = x';
    features = feature_map(x)';
    
    % Predict accelerations with model
    accelerations = model * features;
    
    % Compute new velocities and new state features
    angle = (s(3) + accelerations(3) * dt) * dt;
    Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    s_next(1:2) = Rot * (s(1:2) + accelerations(1:2) * dt);
    if s_next(1) < 0
        s_next(1) = 0;
    end
    s_next(3) = s(3) + accelerations(3) * dt;
    
    s_next(4:6) = phi(s(1:3) * dt, map, s(4:6));
end

