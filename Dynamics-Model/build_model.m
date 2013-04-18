function model = build_model(trajectory, accelerations)
%BUILD_MODEL Summary of this function goes here
%   Detailed explanation goes here
    
    lambda = 0;
    X = feature_map([trajectory.U trajectory.S]);
    Y = accelerations;
    model = zeros(3, size(X,2));
    
    % Select which features to use for regression
    features = zeros(size(X,2), 3);

    features(1, [9 5 12 13 22 23 32 33]) = 1;
    features(2, [6 8]) = 1;
    features(3, [4 7 8 18 28 10 20 30 40 50]) = 1;

    features = logical(features);
    
    % \ddot{x}
    Xprime = X(:,features(1,:));
    theta = pinv(Xprime' * Xprime + lambda * eye(size(Xprime,2))) * Xprime' * Y(:,1);
    model(1,features(1,:)) = theta';
    
    % \ddot{y}
    Xprime = X(:,features(2,:));
    theta = pinv(Xprime' * Xprime + lambda * eye(size(Xprime,2))) * Xprime' * Y(:,2);
    model(2,features(2,:)) = theta';
    
    % \ddot{\omega}
    Xprime = X(:,features(3,:));
    theta = pinv(Xprime' * Xprime + lambda * eye(size(Xprime,2))) * Xprime' * Y(:,3);
    model(3,features(3,:)) = theta';
end

