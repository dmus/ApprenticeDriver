addpath('../Data', '../Yawrate-Estimation', '../General', '../Utilities/LIBSVM');

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

m = size(Y_train,1);
m_test = size(Y_test,1);

clear model;

trn_data.X = X_train;
trn_data.y = Y_train(:,1);

tst_data.X = X_test;
tst_data.y = Y_test(:,1);

[trn_data, tst_data, jn2] = scaleSVM(trn_data, tst_data, trn_data, 0, 1);

param.s = 3; 					% epsilon SVR
param.C = max(trn_data.y) - min(trn_data.y);	% FIX C based on Equation 9.61
param.t = 2; 					% RBF kernel
param.gset = 2.^[-7:7];				% range of the gamma parameter
param.eset = [0:5];				% range of the epsilon parameter
param.nfold = 5;				% 5-fold CV

Rval = zeros(length(param.gset), length(param.eset));

for i = 1:param.nfold
	% partition the training data into the learning/validation
	% in this example, the 5-fold data partitioning is done by the following strategy,
	% for partition 1: Use samples 1, 6, 11, ... as validation samples and
	%			the remaining as learning samples
	% for partition 2: Use samples 2, 7, 12, ... as validation samples and
	%			the remaining as learning samples
	%   :
	% for partition 5: Use samples 5, 10, 15, ... as validation samples and
	%			the remaining as learning samples

	data = [trn_data.y, trn_data.X];
	[learn, val] = k_FoldCV_SPLIT(data, param.nfold, i);
	lrndata.X = learn(:, 2:end);
	lrndata.y = learn(:, 1);
	valdata.X = val(:, 2:end);
	valdata.y = val(:, 1);

	for j = 1:length(param.gset)
		param.g = param.gset(j);

		for k = 1:length(param.eset)
			param.e = param.eset(k);
			param.libsvm = ['-s ', num2str(param.s), ' -t ', num2str(param.t), ...
					' -c ', num2str(param.C), ' -g ', num2str(param.g), ...
					' -p ', num2str(param.e)];

			% build model on Learning data
			model = svmtrain(lrndata.y, lrndata.X, param.libsvm);

			% predict on the validation data
			[y_hat, Acc, projection] = svmpredict(valdata.y, valdata.X, model);

			Rval(j,k) = Rval(j,k) + mean((y_hat-valdata.y).^2);
		end
	end

end

Rval = Rval ./ (param.nfold);

[v1, i1] = min(Rval);
[v2, i2] = min(v1);
optparam = param;
optparam.g = param.gset( i1(i2) );
optparam.e = param.eset(i2);

optparam.libsvm = ['-s ', num2str(optparam.s), ' -t ', num2str(optparam.t), ...
		' -c', num2str(optparam.C), ' -g ', num2str(optparam.g), ...
		' -p ', num2str(optparam.e)];

model = svmtrain(trn_data.y, trn_data.X, optparam.libsvm);

% MSE for test samples
[y_hat, Acc, projection] = svmpredict(tst_data.y, tst_data.X, model);
MSE_Test = mean((y_hat-tst_data.y).^2);
NRMS_Test = sqrt(MSE_Test) / std(tst_data.y);

% MSE for training samples
[y_hat, Acc, projection] = svmpredict(trn_data.y, trn_data.X, model);
MSE_Train = mean((y_hat-trn_data.y).^2);
NRMS_Train = sqrt(MSE_Train) / std(trn_data.y);