function show_map(map)
%SHOW_MAP Plot a map of a track (segment).
%   MAP is plotted by reconstructing the track axis point by point.
    
    % X contains the absolute coordinates
    X = zeros(size(map,1)+1,3);
    
    for i = 1:size(map,1);
        X(i+1,1) = X(i,1) + cos(X(i,3)) * map(i,2);
        X(i+1,2) = X(i,2) + sin(X(i,3)) * map(i,2);
        X(i+1,3) = X(i,3) + map(i,3);
    end

    % Plot positions
    plot(X(:,1), X(:,2), '-r');
end
