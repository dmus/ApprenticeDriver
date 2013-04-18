%% Build a model
if ~exist('data.mat','file')
    [X,Y] = build_dataset(10,false);

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

model = zeros(3, size(X_trainFeatures,2));

% Feature indices to use for regression
features = zeros(size(X_trainFeatures,2), 3);

features(1, [9 5 12 13 22 23 32 33]) = 1;
features(2, [6 8]) = 1;
features(3, [4 7 8 18 28 10 20 30 40 50]) = 1;

k = size(X_train,2) / 5;

features = logical(features);

% Three regression problems
% for i = 1:3
%     y_train = Y_train(:,i);
% 
%     % Disable some features
%     X_temp = X_trainFeatures(:,features(i,:));
%     
%     % Closed form linear regression
%     %theta = pinv(X_trainFeatures' * X_trainFeatures) * X_trainFeatures' * y_train;
%     theta = X_temp \ y_train;
%     
%     % For ridge regression
%     % theta = pinv(X_trainFeatures' * X_trainFeatures + lambda * speye(m)) * X_trainFeatures' * y_train;
%     
%     model(i,features(i,:)) = theta';
%     
%     % Test model
%     %y_train = Y_train(:,i);
%     y_test = Y_test(:,i);
%     
%     % Cost for a particular choice of theta
%     J(i) = (1/(m)) * sum(((X_trainFeatures * model(i,:)') - y_train) .^ 2);
%     J_test(i) = (1/(m_test)) * sum(((X_testFeatures * model(i,:)') - y_test) .^ 2);
%     
%     signsCorrectTrain = sum(sign(X_trainFeatures * model(i,:)') == sign(y_train)) / m;
%     signsCorrectTest = sum(sign(X_testFeatures * model(i,:)') == sign(y_test)) / m_test;
%     
%     fprintf('%f signs correct train\n', signsCorrectTrain);
%     fprintf('%f signs correct test\n', signsCorrectTest);
%     fprintf('%f mse train\n', J(i));
%     fprintf('%f mse test\n', J_test(i));
% end



%% Part two

H = 10000;
if ~exist('r', 'var')
    r = build_trajectory('E-Track-2_MrRacer.mat', 10, H);
end
x = 1:H;

X = feature_map([r.U r.S]);
lambda = 0;

% Compute accelerations
Y = zeros(H-1, 3);
i = 1;
for t = 1:H-1
    dt = r.T(t+1) - r.T(t);
    angle = r.S(t+1,3) * dt;
    R = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
    Y(t,1:2) = ((R * r.S(t+1,1:2)')' - r.S(t,1:2)) / dt;
end
Y(:,3) = (r.S(2:end,3)-r.S(1:end-1,3)) / dt;


%% Yawrate

% First a plot with old parameters
plot(x(1:end-1),Y(:,3),'-bs',x, r.S(:,8),'-rs', x, r.S(:,3), '-gs','Markersize', 2);
hold on;

plot(x, X * model(3,:)', '-ks', 'Markersize', 2);
hold off;

% Try a new yawrate model

% Train on specific model
window = 3205:3500;
window = 1:9999;
Xprime = X(window,features(3,:));
Yprime = Y(window,:);
disp('Yawrate theta');
theta = pinv(Xprime' * Xprime + lambda * eye(size(Xprime,2))) * Xprime' * Yprime(:,3)
model(3,features(3,:)) = theta';
hprime = Xprime * theta;

mse = ((hprime - Yprime(:,3))' * (hprime - Yprime(:,3))) / length(window)

% Plot trajectory
x = 1:size(X,1);
h = X(:,features(3,:)) * theta;

mse = ((h(1:end-1) - Y(:,3))' * (h(1:end-1) - Y(:,3))) / (length(h)-1)
signsCorrect = sum(sign(h(1:end-1)) == sign(Y(:,3))) / (length(h)-1)

figure;
plot(x(1:end-1),Y(:,3),'-bs',x, r.S(:,8),'-rs', x, r.S(:,3), '-gs','Markersize', 2);
hold on;

plot(x, h, '-ks', 'Markersize', 2);
hold off;

% Plot training window
figure;
plot(window,Yprime(:,3),'-bs',window, r.S(window,8),'-rs', window, r.S(window,3), '-gs','Markersize', 2);
hold on;

plot(window, Xprime * theta, '-ks', 'Markersize', 2);
hold off;

%% Longitudinal acceleration
window = 3730:3950;
Xprime = X(window,features(1,:));
Yprime = Y(window,:);
disp('Longitudinal theta');
theta = pinv(Xprime' * Xprime + lambda * eye(size(Xprime,2))) * Xprime' * Yprime(:,1)
model(1,features(1,:)) = theta';
hprime = Xprime * theta;
h = X(:,features(1,:)) * theta;
x = 1:size(X,1);

mse = ((hprime - Yprime(:,1))' * (hprime - Yprime(:,1))) / length(window)

figure;
% plot(window, r.U(window,1),'-rs', window, r.S(window,1) ./ 100, '-gs', window, r.S(window,2) .* r.S(window,3), '-ys', window,  Yprime(:,1), '-bs', 'Markersize', 1);
plot(x, r.U(:,1),'-rs', x, r.S(:,1) ./ 10, '-gs', x, r.S(:,2) .* r.S(:,3), '-ys', x(1:end-1),  Y(:,1), '-bs', 'Markersize', 1);
hold on;

% plot(window, Xprime * theta, '-ks', 'Markersize', 2);
x = 1:size(X,1);
plot(x, h, '-ks', 'Markersize', 2);
hold off;

%% Lateral acceleration
window = 3200:3700;
Xprime = X(window,features(2,:));
Yprime = Y(window,:);
disp('Lateral theta');
theta = pinv(Xprime' * Xprime + lambda * eye(size(Xprime,2))) * Xprime' * Yprime(:,2)
%theta = [-0.1; -0.31]; % TODO fix
model(2,features(2,:)) = theta';
h = X(:,features(2,:)) * theta;

figure;
% plot(window, r.U(window,1),'-rs', window, r.S(window,1) ./ 100, '-gs', window, r.S(window,2) .* r.S(window,3), '-ys', window,  Yprime(:,1), '-bs', 'Markersize', 1);
delta = (r.S(2:end-2,1) .* r.S(2:end-2,3)) - (r.S(1:end-3,1) .* r.S(1:end-3,3));
delta = [0; delta];
plot(x, r.S(:,2), '-gs', x, r.S(:,1) .* r.S(:,3), '-rs', x(1:end-1),  Y(:,2), '-bs', 'Markersize', 1);
hold on;

x = 1:size(X,1);
plot(x, h, '-ks', 'Markersize', 2);
hold off;