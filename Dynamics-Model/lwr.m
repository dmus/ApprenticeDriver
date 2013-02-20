addpath('../Data', '../Yawrate-Estimation', '../General');

if ~exist('data.mat','file')
    [X,Y] = build_dataset(3,true);

    % Random split into training and test set
    m = size(X,1);
    permutation = randperm(m);
    split = floor(0.7*m);

    X_train = X(permutation(1:split),:);
    Y_train = Y(permutation(1:split),:);

    X_test = X(permutation(split+1:end),:);
    Y_test = Y(permutation(split+1:end),:);

    save('data.mat', 'X_train', 'Y_train', 'X_test', 'Y_test');
else
    load('data.mat');
end

% Map atributes to features
X_train = feature_map(X_train);
X_test = feature_map(X_test);
m = size(Y_train,1);

% Normalize data
average = mean(X_train)';
standardDev = std(X_train)';

% Replace zeros to prevent dividing by zero
standardDev(standardDev == 0) = 1;

X_trainNormalized = bsxfun(@rdivide, bsxfun(@minus, X_train, average'), standardDev');
X_testNormalized = bsxfun(@rdivide, bsxfun(@minus, X_train, average'), standardDev');

tau = 100;
J = 0;
for i = 1:size(Y_test,1)
    x = X_test(i,:)';
    y = X_test(i,3);
    
    T = X_trainNormalized' - repmat(x, 1, m);
    s = exp((-sum(T.^2)) / (2 * tau^2));
    
    W = sparse(1:m,1:m,s);
    %W = exp(-(X_trainNormalized - repmat(x', m, 1)) * (X_trainNormalized' - repmat(x, 1, m)) / (2 * t^2));
    
    % Closed form linear regression
    theta = pinv(X_trainNormalized' * W * X_trainNormalized) * X_trainNormalized' * W * y_train;
    h = theta' * x;
    
    % Cost for a particular choice of theta
    J = J + (h-y)^2;
    
    fprintf('%f %f\n', y, h);
end
J = J / (2 * size(Y_test,1));

% Predictions for some random examples