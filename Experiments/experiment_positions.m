% Load trajectory and corresponding accelerations
load('trajectory.mat');

% Divide in train and test set
splitIndex = ceil(0.7*size(trajectory.S,1));
trainWindow = 1:splitIndex-1;
testWindow = splitIndex:size(accelerations,1);

% Ignore last state-action, because no corresponding accelerations
% trajectory.S = trajectory.S(1:end-1,:);
% trajectory.U = trajectory.U(1:end-1,:);
% trajectory.T = trajectory.T(1:end-1,:);

trainTrajectory.S = trajectory.S(trainWindow,:);
trainTrajectory.U = trajectory.U(trainWindow,:);
trainTrajectory.T = trajectory.T(trainWindow,:);

testTrajectory.S = trajectory.S(testWindow,:);
testTrajectory.U = trajectory.U(testWindow,:);
testTrajectory.T = trajectory.T(testWindow,:);

trainAccelerations = accelerations(trainWindow,:);
testAccelerations = accelerations(testWindow,:);

% Load the dynamics model and the map
load('model.mat');
load('map.mat');

% Predict track position and angle
squaredErrorAcc = zeros(6,1);
signsAcc = zeros(6,1);

for t = 1:length(testWindow)-1
    s = testTrajectory.S(t,:)';
    u = testTrajectory.U(t,:)';
    dt = testTrajectory.T(t+1)-testTrajectory.T(t);
    h = f(s, u, dt, model, map);
    y = testTrajectory.S(t+1,:)';
    
    error = (y(1:6) - h(1:6)).^2;
    squaredErrorAcc = squaredErrorAcc + error;
    
    % Increase/Decrease
    delta = testTrajectory.S(t+1,1:6)' - testTrajectory.S(t,1:6)';
    deltaPredicted = h(1:6) - testTrajectory.S(t,1:6)';
    
    signsAcc = signsAcc + (sign(delta) == sign(deltaPredicted));
end

squaredErrorAcc = zeros(6,10);
signsAcc = zeros(6,10);

for t = 1:length(testWindow)-10
    s = testTrajectory.S(t,:)';
    u = testTrajectory.U(t,:)';
    
    for k = 1:10
        % Prediction for t+k
        dt = testTrajectory.T(t+k)-testTrajectory.T(t+k-1);
        h = f(s, u, dt, model, map);
        y = testTrajectory.S(t+k,:)';
        
        % Compute error
        error = (y(1:6) - h(1:6)).^2;
        squaredErrorAcc(:,k) = squaredErrorAcc(:,k) + error;
        
        % Increase/Decrease
        delta = testTrajectory.S(t+k,1:6)' - testTrajectory.S(t+k-1,1:6)';
        deltaPredicted = h(1:6) - testTrajectory.S(t+k-1,1:6)';
    
        signsAcc(:,k) = signsAcc(:,k) + (sign(delta) == sign(deltaPredicted));
        
        % For next iteration
        s = h;
        u = testTrajectory.U(t+k,:)';
    end
    
end

% Mean squared error
mse = squaredErrorAcc ./ (length(testWindow)-1);

% Check increase/decrease
signsCorrect = signsAcc ./ (length(testWindow)-1);

t = 0.02:0.02:0.2;
titels = {'\dot{x}', '\dot{y}', '\dot{\omega}', 'p', 'q', '\alpha'};
for i = 1:6
    subplot(2,3,i);
    plot(t,mse(i,:));
    xlabel('t(s)');
    title(titels(i));
end