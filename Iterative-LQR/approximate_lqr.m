function [A,B,Q,R] = approximate_lqr(reference, f, g, alpha, model, map, bias)
%APPROXIMATE_LQR Approximate a standard linear-time varying LQR problem.
%   Dynamics and cost matrices are approximated with Taylor expansions.
    
    S = reference.S;
    U = reference.U;
    times = reference.T;
    
    % Horizon
    H = size(S,1);
    
    % State cost function
    %g = @g_general;
    
    n_states = size(S,2);
    n_inputs = size(U,2);
    
    % Parameter for finite difference methods
    my_eps = 0.01;
    
    for t = 1:H-1
        s_ref = S(t,:)';
        s_ref_tplus1 = S(t+1,:)';
        
        % Control input and time difference
        u_ref = U(t,:)';
        dt = times(t+1) - times(t);
        
        % Linearize dynamics around reference trajectory
        f_t = @(s_ref, u_ref, dt, model, map) f(s_ref, u_ref, dt, model, map) + bias(t,:)';
        [A{t}, B{t}, c{t}] = linearize_dynamics(f_t, s_ref, u_ref, dt, my_eps, s_ref_tplus1, model, map);
        
        % Transform affine system into standard LQR format
        A{t} = [A{t} c{t}; zeros(1, n_states) 1];
        Temp = zeros(n_states + 1, n_inputs + 1);
        Temp(1:size(B{t},1),1:size(B{t},2)) = B{t};
        B{t} = Temp;
        
        % Approximate costs in quadratic form around reference trajectory
        [Q{t}, R{t}] = quadraticize_costs(g, @h, s_ref, u_ref, my_eps);
        
        % Penalize deviations from reference trajectory
        Q{t} = (1 - alpha) * Q{t} + alpha * [eye(n_states) zeros(n_states,1); zeros(1,n_states + 1)];
        %Q{t} = zeros(size(Q{t}));
        R{t} = (1 - alpha) * R{t} + alpha * [eye(n_inputs) zeros(n_inputs,1); zeros(1,n_inputs + 1)];
        
        % Augment matrices
        Aprime{t} = [A{t} B{t};
                     zeros(n_inputs+1,n_states+1) eye(n_inputs+1)];
                 
        Bprime{t} = [B{t}(1:n_states+1, 1:n_inputs);
                     eye(3,2)];
        
        if t == 1
            Qprime{t} = [Q{t} zeros(n_states+1, n_inputs+1);
                         zeros(n_inputs+1, n_states+1) zeros(size(R{t}))];
        else
            Qprime{t} = [Q{t} zeros(n_states+1, n_inputs+1);
                         zeros(n_inputs+1, n_states+1) R{t-1}];
        end
        
        % Matrix to penalize change in control inputs
        Rprime{t} = 1 * eye(2);
                 
    end

    Q{H} = quadraticize_costs(g, [], S(H,:)', [], my_eps);
    Q{H} = (1 - alpha) * Q{H} + alpha * [eye(n_states) zeros(n_states,1); zeros(1, n_states + 1)];
    
    Qprime{H} = [Q{H} zeros(n_states+1, n_inputs+1);
                 zeros(n_inputs+1, n_states+1) R{H-1}];
      
    A = Aprime;
    B = Bprime;
    Q = Qprime;
    R = Rprime;
end

