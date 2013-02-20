clear;
load('data.mat');

X = rand(10000,10);
X = [X X.^2 X.^3 X.^4 X.^5 X.^6 X.^5 X.^4 X.^10 X.^9];
X = feature_map(X_train);
Xtest = feature_map(X_test);
%X = [X];
y = Y_train(:,3);
ytest = Y_test(:,3);

%theta = pinv(X' * X) * X' * y;
theta = X\y; 
 
 J = (1/(2*length(y))) * sum(((X * theta) - y) .^ 2)
 J_test = (1/(2*length(ytest))) * sum(((Xtest * theta) - ytest) .^ 2)
 %J = sum((X * theta) - y);
 %theta
 
%  f = @(x) (1/(2*length(y))) * sum(((X * x) - y) .^ 2);
%  theta_ = fminsearch(f, zeros(size(theta)));
%  
%  [theta theta_]