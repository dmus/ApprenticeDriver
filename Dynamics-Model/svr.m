addpath('../Data', '../Yawrate-Estimation', '../General', '../Utilities/libsvm/windows');

[D_train mu sigma] = normalize([X_train Y_train]);
X_trainNormalized = D_train(:,1:15);
Y_trainNormalized = D_train(:,16:18);

D_test = normalize([X_test, Y_test]);
X_testNormalized = D_test(:,1:15);
Y_testNormalized = D_test(:,16:18);

model = svmtrain(Y_trainNormalized(:,1), X_trainNormalized, '-s 4 -t 1 -n 0.5 -c 1');
predictions = svmpredict(Y_testNormalized(:,1), Y_testNormalized, model);

h = predictions * sigma(16) + mu(16);
m_test = size(Y_train,1);
J_test = (1/(2*m_test)) * sum((h - Y_test(:,1)) .^2);