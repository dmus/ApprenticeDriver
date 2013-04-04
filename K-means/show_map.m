function show_map(map)
%SHOW_MAP Plot a map of a track (segment).
%   MAP is plotted by reconstructing the track axis point by point.
    
    % X contains the absolute coordinates and angles
    X = zeros(size(map,1)+1,2);

    angle = 0;
    for t = 1:size(map,1)
        angle = angle + map(t,3);
        R = [cos(angle) -sin(angle);sin(angle) cos(angle)];
        X(t+1,1:2) = X(t,1:2)' + R * [map(t,2); 0];
    end
    
    plot(X(:,1), X(:,2), '-r');
end
