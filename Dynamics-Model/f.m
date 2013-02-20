function s_next = f(s, u, dt, model, map)
%F Predicts next state.
%   Detailed explanation goes here
    
    % Make sure limits are respected in control inputs
    u(u > 1) = 1;
    u(u < -1) = -1;

    s_next = s;
    s_next(7:end) = [u; s(1:end-8)]; % ROTATE TO BODY FRAME AT TIME T
    
    % Only velocity features and control inputs to predict accelerations
    s_vel = s;
    m = length(s_vel);
    toTrash = [4:8:m 5:8:m 6:8:m];
    s_vel(toTrash) = [];
    x = [u' s_vel'];
    
    % Rotate previous velocities
    k = length(x) / 5;
    for i = 1:k-1
        angle = x(5) * dt;
        Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
        x(8:9) = (Rot * x(8:9)')';
    end
    
    features = feature_map(x)';
    
    % Predict accelerations with model
    accelerations = model * features;
    
    % Compute new velocities and new state features
    angle = s(3) * dt;
    Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    s_next(1:2) = Rot * (s(1:2) + accelerations(1:2) * dt); 
    s_next(3) = s(3) + accelerations(3) * dt;
    
    % s_next(4:6) = phi(s(1:3), map, s(4:6));
end

