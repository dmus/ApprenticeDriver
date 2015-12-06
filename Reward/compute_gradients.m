function [A,B,invB,dxdu,d2xdudu] = compute_gradients(trajectory, model, map)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%f(s, u, dt, model, map);
dt = 0.02;

Dx = 6;
Du = 2;
T = size(trajectory.S,1);

A = zeros(Dx,Dx,T);
B = zeros(Dx,Du,T);

s_eps = 0.1;
u_eps = 0.1;

for t = 1:T
    s = trajectory.S(t,1:Dx)';
    u = trajectory.U(t,:)';
    
    for i = 1:Dx
        s_minus = s;
        s_minus(i) = s_minus(i) - s_eps;

        s_plus = s;
        s_plus(i) = s_plus(i) + s_eps;

        A(:,i,t) = (f(s_plus,u,dt,model,map)-f(s_minus,u,dt,model,map)) / (2*s_eps);
    end
    
    for i = 1:Du
        u_minus = u;
        u_minus(i) = u_minus(i) - u_eps;

        u_plus = u;
        u_plus(i) = u_plus(i) + u_eps;

        B(:,i,t) = (f(s,u_plus,dt,model,map)-f(s,u_minus,dt,model,map)) / (2*u_eps);
    end
end

% Compute inverses.
if nargout >= 4,
    invB = zeros(Du,Dx,T);
    for t = 1:T,
        invB(:,:,t) = pinv(B(:,:,t));
    end
end

% Now build the Jacobian out of these matrices.
dxdu = zeros(Du*T,Dx*T);
uidx = (0:T:(T*(Du-1)));
xidx = (0:T:(T*(Dx-1)));
for c=1:T,
    % First, compute the top part of this block row.
    for r=1:(c-1),
        dxdu(r + uidx, c + xidx) = dxdu(r + uidx, (c-1) + xidx)*A(:,:,c)';
    end;
    % Now write the diagonal block.
    dxdu(c + uidx, c + xidx) = B(:,:,c)';
end;

% The Hessian is zero.
% Note that this is not actually the case, but we'll leave it like this and
% use the locally linear approximation.
if nargout >= 6,
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
end
