trajectory = build_trajectory('Wheel-2_MrRacer.mat', 6);
%trajectory = trajectory1;
accelerations = compute_accelerations(trajectory);

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

model = build_model(trainTrajectory, trainAccelerations);

trainStats = test_model(model, trainTrajectory, trainAccelerations);
testStats = test_model(model, testTrajectory, testAccelerations);