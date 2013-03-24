function [D_normalized, mu, sigma] = normalize(D, mu, sigma)
%NORMALIZE Summary of this function goes here
%   Detailed explanation goes here
    if nargin == 1
        mu = mean(D);
        sigma = std(D);
    end

    D_normalized = bsxfun(@rdivide, bsxfun(@minus, D, mu), sigma);
end

