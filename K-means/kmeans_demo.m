%% Configuration
rfSize = 50;
numCentroids = 100;
whitening=false;

addpath minFunc;

%% Load training data
fprintf('Loading training data...\n');
% [trials] = extract_trials('Wheel-2_SimpleDriver.mat');
% times = compute_discretized_times(trials{2}.sensors);

maps{1} = build_map(trials{2}.S, times);

% Double training data artificially by mirroring
maps{2} = [map(:,1) map(:,2) -1 * map(:,3)];



% extract random patches
numPatches = (size(maps{1},1) - rfSize + 1) * length(maps);
patches = zeros(numPatches, rfSize*2);
first = 1;
last = first + rfSize - 1;
mapIndex = 1;

for i = 1:numPatches
    patch = maps{mapIndex}(first:last,2:3);
    patches(i,:) = patch(:)';
    
    first = first + 1;
    last = last + 1;
    
    % Check if we have to go to the next map
    if last > size(maps{mapIndex},1)
        first = 1;
        last = first + rfSize - 1;
        mapIndex = mapIndex + 1;
    end
end

% % normalize for contrast
% meanOverDistances = mean(patches(:,1:2:end-1));
% meanOverRotations = 0;
% stdOverDistances = sqrt(var(patches(:,2:2:end))+10);
% stdOverRotations = sqrt(var(patches(:,2:2:end))+10);
% patches(:,1:2:end-1) = bsxfun(@rdivide, bsxfun(@minus, patches(:,1:2:end-1), meanOverDistances), stdOverDistances);
% patches(:,2:2:end) = bsxfun(@rdivide, bsxfun(@minus, patches(:,2:2:end), meanOverRotations), stdOverRotations);

% whiten
if (whitening)
  C = cov(patches);
  M = mean(patches);
  [V,D] = eig(C);
  P = V * diag(sqrt(1./(diag(D) + 0.1))) * V';
  patches = bsxfun(@minus, patches, M) * P;
end

% run K-means
centroids = run_kmeans(patches, numCentroids, 100);
base = zeros(rfSize,3);
for i = 1:numCentroids
    base(:,2:3) = reshape(centroids(i,:), rfSize, 2);
    subplot(10,10,i);
    show_map(base);
    axis([0 50 -3 3])
end
show_centroids(centroids, rfSize); drawnow;

% extract training features
trainXC = extract_features(trainX, centroids, rfSize, CIFAR_DIM);

% standardize data
trainXC_mean = mean(trainXC);
trainXC_sd = sqrt(var(trainXC)+0.01);
trainXCs = bsxfun(@rdivide, bsxfun(@minus, trainXC, trainXC_mean), trainXC_sd);
trainXCs = [trainXCs, ones(size(trainXCs,1),1)];

% train classifier using SVM
C = 100;
theta = train_svm(trainXCs, trainY, C);

[val,labels] = max(trainXCs*theta, [], 2);
fprintf('Train accuracy %f%%\n', 100 * (1 - sum(labels ~= trainY) / length(trainY)));

%%%%% TESTING %%%%%

%% Load CIFAR test data
fprintf('Loading test data...\n');
f1=load([CIFAR_DIR '/test_batch.mat']);
testX = double(f1.data);
testY = double(f1.labels) + 1;
clear f1;

% compute testing features and standardize
if (whitening)
  testXC = extract_features(testX, centroids, rfSize, CIFAR_DIM, M,P);
else
  testXC = extract_features(testX, centroids, rfSize, CIFAR_DIM);
end
testXCs = bsxfun(@rdivide, bsxfun(@minus, testXC, trainXC_mean), trainXC_sd);
testXCs = [testXCs, ones(size(testXCs,1),1)];

% test and print result
[val,labels] = max(testXCs*theta, [], 2);
fprintf('Test accuracy %f%%\n', 100 * (1 - sum(labels ~= testY) / length(testY)));

