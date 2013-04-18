function stats = test_model(model, trajectory, accelerations)
%TEST_MODEL Summary of this function goes here
%   Detailed explanation goes here
    
    X = feature_map([trajectory.U trajectory.S]);
    Y = accelerations;
    H = X * model';
    
    m = size(X,1);
    
    % Plotting indices
    x = 1:size(Y,1);
    
    % \ddot{x}
    stats.x.mse = ((H(:,1) - Y(:,1))' * (H(:,1) - Y(:,1))) / m;
    stats.x.signs = sum(sign(H(:,1)) == sign(Y(:,1))) / m;
    
    figure;
    plot(x, trajectory.U(:,1),'-rs', x, trajectory.S(:,1) ./ 10, '-gs', x, trajectory.S(:,2) .* trajectory.S(:,3), '-ms', x,  Y(:,1), '-bs', 'Markersize', 1);
    hold on;
    plot(x, H(:,1), '-ks', 'Markersize', 2);
    
    h = legend('$u_1$', '$\dot{x}$', '$\dot{y}\dot{\omega}$', '$\ddot{x}$');
    set(h,'Interpreter','latex');
    set(h,'FontSize',20);
    hold off;

    % \ddot{y}
    stats.y.mse = ((H(:,2) - Y(:,2))' * (H(:,2) - Y(:,2))) / m;
    stats.y.signs = sum(sign(H(:,2)) == sign(Y(:,2))) / m;
    
    figure;
    plot(x, trajectory.S(:,2), '-gs', x, trajectory.S(:,1) .* trajectory.S(:,3), '-rs', x,  Y(:,2), '-bs', 'Markersize', 1);
    hold on;

    plot(x, H(:,2), '-ks', 'Markersize', 2);
    
    h = legend('$\dot{y}$', '$\dot{x} \dot{\omega}$', '$\ddot{y}$');
    set(h,'Interpreter','latex');
    set(h,'FontSize',20);
    hold off;
    
    % \ddot{\omega}
    stats.omega.mse = ((H(:,3) - Y(:,3))' * (H(:,3) - Y(:,3))) / m;
    stats.omega.signs = sum(sign(H(:,3)) == sign(Y(:,3))) / m;
    
    figure;
    plot(x,Y(:,3),'-bs',x, trajectory.U(:,2),'-rs', x, trajectory.S(:,3), '-gs','Markersize', 2);
    hold on;

    plot(x, H(:,3), '-ks', 'Markersize', 2);
    
    h = legend('$\ddot{\omega}$', '$u_2$', '$\dot{\omega}$');
    set(h,'Interpreter','latex');
    set(h,'FontSize',20);
    hold off;
end
