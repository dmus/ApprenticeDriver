function f = phi(s, map, current)
%PHI Computes three state features (distance along track axis, distance from track axis, angle with respect to track axis)..
%   S is position (x,y,alpha) relative with respect to origin, MAP is a map
%   of the full track axis and CURRENT is current position and orientation
%   relative to track axis.

    mapSize = size(map,1);

    trackWidth = 12;
    trackLength = 6205.462891;
    % Absolute trackpos
    trackPos = current(2) * (trackWidth / 2);
    alpha = current(3);

    % Compute point on track axis, relative to current position 
    if trackPos > 0 
        track = [abs(trackPos) * cos(alpha - 0.5*pi); abs(trackPos) * sin(alpha - 0.5*pi)];
    else
        track = [abs(trackPos) * cos(alpha + 0.5*pi); abs(trackPos) * sin(alpha + 0.5*pi)];
    end
    
    % Find corresponding part of map
    index = find(map(:,1) > current(1), 1);
    
    % When at the end
    if size(index,1) == 0
        index = 1;
    end
    prev_index = index - 1;
    
    if prev_index == 0
        prev_index = size(map,1);
    end
    
    % P1 and P2 are two points on track axis, distances along track axis
    distance_to_p1 = current(1) - map(prev_index,1);
    distance_to_p2 = map(index,1) - current(1);
    
    if prev_index == size(map,1)
        distance_to_p1 = current(1) + (trackLength - map(end,1));
    end
    
    p1 = track + [distance_to_p1 * cos(alpha + pi); distance_to_p1 * sin(alpha + pi)];
    p2 = track + [distance_to_p2 * cos(alpha); distance_to_p2 * sin(alpha)];
    
    p = s(1:2);
    r = p - p1;
    dp = p2 - p1;
    u = (r' * dp) / (dp' * dp);
    
    firstIndex = prev_index;
    while u > 1
        alpha = alpha + map(prev_index,3);
        p1 = p2;
        p2 = p2 + [map(index,2) * cos(alpha); map(index,2) * sin(alpha)];
        
        r = p - p1;
        dp = p2 - p1;
        u = (r' * dp) / (dp' * dp);
    
        index = index + 1;
        
        if index > size(map,1)
            index = 1;
        end
        
        prev_index = index - 1;
        if prev_index == 0
            prev_index = size(map,1);
        end
    end
    
    f = zeros(3,1);
    
    % Distance from track axis
    relative_cte = (r(2) * dp(1) - r(1) * dp(2)) / (dp' * dp);
    absolute_cte = relative_cte * sqrt(dp' * dp);
    f(2) = absolute_cte / (0.5 * trackWidth);
    
    % Angle with respect to track axis
    f(3) = (s(3) - (alpha + u * map(prev_index,3)));
    
    % Compute angle track has made
    trackAngle = 0;
    while firstIndex ~= prev_index
        a = 1;
        if current(1) > map(firstIndex,1)
            a = (current(1) - map(firstIndex,1)) / map(firstIndex,2);
        end
        
        % Special case
        if current(1) < map(1,1)
            covered = trackLength - map(end,1) + current(1);
            a = covered / map(end,2);
        end
        
        trackAngle = trackAngle + a * map(firstIndex,3);
        firstIndex = firstIndex + 1;
        if firstIndex > size(map,1)
            firstIndex = 1;
        end
    end
    
    trackAngle = trackAngle + u * map(prev_index,3);
    f(3) = current(3) - s(3) + trackAngle;
    % Distance to go along track axis
    %f(1) = map(prev_index,1) + u * (map(index,1) - map(prev_index,2));
    f(1) = map(prev_index,1) + u * map(prev_index,2);
    
    if f(1) >= trackLength
        f(1) = f(1) - trackLength;
    end
    f(1) = 0;
end

