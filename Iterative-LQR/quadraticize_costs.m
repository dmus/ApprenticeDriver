function [Q, R] = quadraticize_costs(g, h, x_ref, u_ref, my_eps)
%APPROXIMATECOST Summary of this function goes here
%   Detailed explanation goes here

    x_eps = my_eps;
    u_eps = my_eps;

    % Constant term
    q_0 = g(x_ref);
    
    % Compute g_x with finite difference method
    q = zeros(size(x_ref));
    for i = 1:length(x_ref)
        x_plus = x_ref;
        x_plus(i) = x_plus(i) + x_eps;
        x1_plus = g(x_plus);
        
        x_minus = x_ref;
        x_minus(i) = x_minus(i) - x_eps;
        x1_minus = g(x_minus);
        
        q(i) = (x1_plus - x1_minus) / (2*x_eps);
    end

    % Now partial derivative g_{xx}
    Q = zeros(length(x_ref));
    for i = 1:length(x_ref)
        for j = 1:length(x_ref)
            % (i,j) entry in Hessian
            
            % Entry on the diagonal
            if i == j
                x1 = x_ref;
                x1(i) = x1(i) + x_eps;
                
                x3 = x_ref;
                x3(i) = x3(i) - x_eps;
                
                Q(i,j) = (g(x1) - 2*q_0 + g(x3)) / x_eps^2;
            else
                x1 = x_ref;
                x1([i j]) = x1([i j]) + x_eps;

                x2 = x_ref;
                x2([i j]) = x2([i j]) + [x_eps; -x_eps];

                x3 = x_ref;
                x3([i j]) = x3([i j]) + [-x_eps; x_eps];

                x4 = x_ref;
                x4([i j]) = x4([i j]) + [-x_eps; -x_eps];

                Q(i,j) = (g(x1) - g(x2) - g(x3) + g(x4)) / (4 * x_eps * x_eps);
            end
        end
    end
    
    % To quadratic form
    Q = [0.5*Q 0.5*q;0.5*q' q_0];
    
    if nargout <= 1
        return;
    end
    
    % Now the same for the R matrix
    % Constant term
    r_0 = h(u_ref);
    
    % Compute h_u with finite difference method
    r = zeros(size(u_ref));
    for i = 1:length(u_ref)
        u_plus = u_ref;
        u_plus(i) = u_plus(i) + u_eps;
        u1_plus = h(u_plus);
        
        u_minus = u_ref;
        u_minus(i) = u_minus(i) - u_eps;
        u1_minus = h(u_minus);
        
        r(i) = (u1_plus - u1_minus) / (2*u_eps);
    end

    % Now partial derivative h_{uu}
    R = zeros(length(u_ref));
    for i = 1:length(u_ref)
        for j = 1:length(u_ref)
            % (i,j) entry in Hessian
            
            % Entry on the diagonal
            if i == j
                u1 = u_ref;
                u1(i) = u1(i) + u_eps;
                
                u3 = u_ref;
                u3(i) = u3(i) - u_eps;
                
                R(i,j) = (h(u1) - 2*r_0 + h(u3)) / u_eps^2;
            else
                u1 = u_ref;
                u1([i j]) = u1([i j]) + u_eps;

                u2 = u_ref;
                u2([i j]) = u2([i j]) + [u_eps; -u_eps];

                u3 = u_ref;
                u3([i j]) = u3([i j]) + [-u_eps; u_eps];

                u4 = u_ref;
                u4([i j]) = u4([i j]) + [-u_eps; -u_eps];

                R(i,j) = (h(u1) - h(u2) - h(u3) + h(u4)) / (4 * u_eps * u_eps);
            end
        end
    end
    
    % To quadratic form
    R = [0.5*R 0.5*r;0.5*r' r_0];
end


