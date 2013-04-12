function [X,h] = show_map(map, from, to)
%SHOW_MAP Plot a map of a track (segment).
%   MAP is plotted by reconstructing the track axis point by point. X
%   contains the computed points in an absolute coordinate frame.
    
    TRACKWIDTH = 12;

    % X contains the absolute coordinates and angles
    X = zeros(size(map,1)+1,3);
    L = zeros(size(X,1),2);
    R = zeros(size(X,1),2);
    
    L(1,:) = [0, .5 * TRACKWIDTH];
    R(1,:) = [0, -.5 * TRACKWIDTH];
    
    angle = 0;
    for t = 1:size(map,1)
        angle = angle + map(t,3);
        Rot = [cos(angle) -sin(angle);sin(angle) cos(angle)];
        X(t+1,1:2) = X(t,1:2)' + Rot * [map(t,2); 0];
        X(t+1,3) = X(t,3) + map(t,3);
        
        % Left edge
        beta = 0.5 * pi + angle;
        Rot = [cos(beta) -sin(beta);sin(beta) cos(beta)];
        L(t+1,1:2) = X(t+1,1:2)' + Rot * [0.5 * TRACKWIDTH; 0];
        
        % Right edge
        beta = -0.5 * pi + angle;
        Rot = [cos(beta) -sin(beta);sin(beta) cos(beta)];
        R(t+1,1:2) = X(t+1,1:2)' + Rot * [0.5 * TRACKWIDTH; 0];
    end
    
    h = figure;
    plot(X(:,1), X(:,2), '-r');
    hold on;
    A = [L(:,1); flipud(R(:,1))];
    B = [L(:,2); flipud(R(:,2))];
    
    p = fill(A,B,'b');
    set(p,'FaceAlpha',0.2);
end
