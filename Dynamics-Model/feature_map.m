function X_phi = feature_map(X)
%FEATURE_MAP Replace input attributes by input features.
%   X is transformed to X_phi.

    m = size(X,1);
    k = size(X,2) / 8; % Determine number of state-action pairs
    numFeatures = 10; % Number of features per state-action pair
    
    X_phi = zeros(m,k*numFeatures);
    
    for i = 0:k-1
        first = (i*numFeatures)+1;
        last = first+numFeatures-1;
        j = i*8;
        
        if i < k-1
            steeringDelta = X(:,j+2) - X(:,j+2+8);
        else
            steeringDelta = zeros(size(X,1),1);
        end
        
        if i < k-1;
            prevYawAcc = X(:,j+6) - X(:,j+6+8);
        else
            prevYawAcc = zeros(size(X,1),1);
        end
%         if i < k-2
%             steeringDeltaDelta = (X(:,j+2) - X(:,j+2+16));
%         else
%             steeringDeltaDelta = zeros(size(X,1),1);
%         end
        
        X_phi(:,first:last) = [max(X(:,j+1),0)...       % Gas
                               -1 * min(X(:,j+1),0)...        % Brake
                               X(:,[j+2 j+3 j+4 j+5])... % Steering, \ddot{x}, \ddot{y}, \ddot{\omega}
                               X(:,j+3) .* X(:,j+5)...  % \dot{x} * \dot{\omega}
                               X(:,j+4) .* X(:,j+5)...     % \dot{y} * \dot{\omega}
                               steeringDelta...
                               prevYawAcc
                               ];
                            
    end
    
    X_phi = [X_phi];
    
    X_phi = [ones(m,1) X_phi];
end