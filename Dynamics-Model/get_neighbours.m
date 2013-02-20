function [neighbours, distances] = get_neighbours(X, ind)
%GET_NEIGHBOURS Summary of this function goes here
%   Detailed explanation goes here

    m = size(X,1);

    % Normalize data
    X = bsxfun(@rdivide, bsxfun(@minus, X, mean(X)), std(X));

    % Query point
    x = X(ind,:);
    
    % Compute weights
    tau = 1;
    w = X - repmat(x, m, 1);
    w = exp((-sum(w.^2,2)) / (2 * tau^2));
    
    [distances, neighbours] = sort(w, 1, 'descend');
end

