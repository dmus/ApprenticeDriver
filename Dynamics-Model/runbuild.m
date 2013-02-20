%% Build a model

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
X_trainFeatures = feature_map(X_train);
X_testFeatures = feature_map(X_test);
m = size(Y_train,1);
m_test = size(Y_test,1);

clear model;

mask = zeros(3,9);
%mask(1, [3 5 6 7 8 9]) = 1;

k = size(X_train,2) / 5;
mask = repmat(mask, 1, k);

mask = logical(mask);
% TODO check linear regression
for i = 1:3
    y_train = Y_train(:,i);

    % Disable some features
    X_temp = X_trainFeatures;
    X_temp(:,mask(i,:)) = 0;
    
    % Closed form linear regression
    %theta = pinv(X_trainFeatures' * X_trainFeatures) * X_trainFeatures' * y_train;
    theta = X_temp \ y_train;
    
    % For ridge regression
    % theta = pinv(X_trainFeatures' * X_trainFeatures + lambda * speye(m)) * X_trainFeatures' * y_train;
    
    model(i,:) = theta';
    
    % Test model
    %y_train = Y_train(:,i);
    y_test = Y_test(:,i);
    
    % Cost for a particular choice of theta
    J(i) = (1/(2*m)) * sum(((X_trainFeatures * theta) - y_train) .^ 2);
    J_test(i) = (1/(2*m_test)) * sum(((X_testFeatures * theta) - y_test) .^ 2);
    
    signsCorrectTrain = sum(sign(X_trainFeatures * theta) == sign(y_train)) / m;
    signsCorrectTest = sum(sign(X_testFeatures * theta) == sign(y_test)) / m_test;
    
    fprintf('%f signs correct train\n', signsCorrectTrain);
    fprintf('%f signs correct test\n', signsCorrectTest);
    fprintf('%f mse train\n', J(i));
    fprintf('%f mse test\n', J_test(i));
end



% Predictions for some random examples