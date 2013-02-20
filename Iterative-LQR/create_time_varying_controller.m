function [K, P] = create_time_varying_controller(A, B, Q, R, Qfinal)
%CREATETIMEVARYINGCONTROLLER Create linear time-varying controller for LQR
%problem.
%   Detailed explanation goes here

% Now let's use the Value Iteration Solution to LQR control problems
% to solve the following optimal control problem:
% min_{x,u} \sum_{t=1}^T x(:,t)'*Q*x(:,t) + u(:,t)'*R*u(:,t) + x(:,T+1)'*Qfinal*x(:,T+1)
% s.t.      x(:,t+1) = A{t}*x(:,t) + B{t}*u(:,t)

% Problem has been defined, let's solve it:

% LQR Value Iteration back-ups to find:
% K{k}, k=1,2,...,T : feedback control for k time steps to-go if in state x equals K{k}*x
% P{k}, k=1,2,...,T : cost-to-go for k time steps to-go if in state x equals x'*P{k}*x

for t = 1:length(R)
    if sum(eig(Q{t}) < 0) 
        warning('Not positive semi-definite Q matrix at time %d', t);
    end
    
    if sum(eig(R{t}) < 0) 
        warning('Not positive semi-definite R matrix at time %d', t);
    end
end

if sum(eig(Qfinal) < 0) 
    warning('Qfinal not positive semi-definite');
end

P_current = Qfinal;
t = length(A);
for k = 1:length(A)
    K_new = -pinv(R{t} + B{t}' * P_current * B{t}) * B{t}' * P_current * A{t};
    P_new = Q{t} + K_new' * R{t} * K_new + (A{t} + B{t} * K_new)' * P_current * (A{t} + B{t} * K_new);
    
    K_current = K_new;
    P_current = P_new;
    
    K{t} = K_current;
    P{t} = P_current;
    
    t = t-1;
end

P{k+1} = Qfinal;
