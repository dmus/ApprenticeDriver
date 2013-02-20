function X_phi = feature_map(X)
%FEATURE_MAP Replace input attributes by input features.
%   X is transformed to X_phi.

    m = size(X,1);
    k = size(X,2) / 5;
    numFeatures = 6; % Number of features per state-action pair
    
    X_phi = zeros(m,k*numFeatures);
    
    for i = 0:k-1
        first = (i*numFeatures)+1;
        last = first+numFeatures-1;
        j = i*k;
        X_phi(:,first:last) = [min(X(:,j+1),0)...       % Gas
                               -1 * max(X(:,j+1),0)...        % Brake
                               X(:,[j+2 j+3 j+4 j+5])... % Steering, \ddot{x}, \ddot{y}, \ddot{\omega}
                               ...%X(:,j+3) .* X(:,j+5)...  % \dot{x} * \dot{\omega}
                               ...%-1 * max(X(:,j+1), 0) .* X(:,j+5)... % Brake * \ddot{\omega}
                               ...%min(X(:,j+1), 0) .* X(:,j+5)...  % Gas * \ddot{\omega}
                               
                               ];
                            
    end
    
    X_phi = [X_phi X_phi.^2 X_phi.^3];
    
    X_phi = [ones(m,1) X_phi];
end