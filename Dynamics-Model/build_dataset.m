function [X,Y] = build_dataset(k, use_symmetry)
%BUILD_DATASET Builds a dataset of states and actions to train a model.
%   Parameter K indicates how many state-action pairs are needed to predict
%   the next state. USE_SYMMETRY indicates whether the dataset has to be doubled
%   artificially by mirroring.
    if nargin < 3
        use_symmetry = false;
    end

    if k < 1
        error('k values < 1 are not allowed');
    end
    
    files = {
        'Wheel-2_SimpleDriver.mat',...
        'E-Track-2_MrRacer.mat',...
        'Wheel-2_MrRacer.mat'
    };

    X = zeros(0, 5 * k);
    Y = zeros(0, 3);
    
    for i = 1:length(files)
        data = load(files{i});
        
        % Remove states and actions before start signal
        sensors = data.States(data.States(:,2) > 0,:);
        actuators = data.Actions(data.States(:,2) > 0,:);
        
        S = compute_states(sensors);
        U = compute_actions(actuators);
        times = compute_discretized_times(sensors);
        
        X_new = zeros(size(S,1) - k + 1, 5 * k);
        Y_new = zeros(size(S,1) - k + 1, 3);
        
        for t = k:size(S,1)-1
            y = zeros(1,3);
            x = zeros(1,5*k);
            
            % First compute the accelerations
            dt = times(t+1) - times(t);
            angle = S(t+1,3) * dt;
            Rot = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
            
            y(1:2) = (Rot * S(t+1,1:2)') - S(t,1:2)';
            y(3) = S(t+1,3) - S(t,3);
            y = y / dt;
            
            % Now the x values
            x(1:5) = [U(t,:) S(t,1:3)];
            
            % Add previous state-action pairs
            angle = 0;
            for j = 1:k-1
                dt = times(t-j+1) - times(t-j);
                angle = angle + S(t-j+1,3) * dt;
                Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
                v = Rot * S(t-j,1:2)';
                x(j*5+1:(j+1)*5) = [U(t-j,:) v' S(t-j,3)];
            end
            
            X_new(t,:) = x;
            Y_new(t,:) = y;
        end
        
        X = [X; X_new];
        Y = [Y; Y_new];
    end

    if use_symmetry
        % Double dataset by mirroring
        mirror = [2 3 5];
        for i = 1:k-1
            mirror = [mirror (i * 5) + [2 3 5]];
        end
        X_mirror = X;
        X_mirror(:,mirror) = -1 * X_mirror(:,mirror);
        Y_mirror = [Y(:,1) -1*Y(:,2) -1*Y(:,3)];
        
        X = [X; X_mirror];
        Y = [Y; Y_mirror];
    end
    
    % Delete rows with nan's
    toTrash = (sum(isnan(X),2) > 0) | (sum(isnan(Y),2) > 0);
    X(toTrash,:) = [];
    Y(toTrash,:) = [];
end

