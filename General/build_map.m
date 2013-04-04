function map = build_map(S, times)
%BUILDMAP Build a map from sensor information.
%   MAP consists of a series of tuples of the form (distanceAlongTrackAxis,
%   distance, rotation). Each tuple described with distance and rotation
%   the rotation and distance to the next point on the track axis.

    TRACKWIDTH = 12;
    trackLength = 6205.462891;
    
    len = size(S,1);
    P = zeros(len,3); % For absolute coordinates
    
    track = zeros(len,2);

    % Compute absolute positions, first position is fixed at (0,0)
    for t = 2:len
        dt = times(t) - times(t-1);
        velocity = 0.5 * (S(t-1,1:2) + S(t,1:2));
        
        % Compute absolute position and orientation
        rotation = P(t-1,3);
        R = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
        
        P(t,1:2) = P(t-1,1:2) + (R * (velocity .* dt)')';
        P(t,3) = P(t-1,3) + S(t,3) * dt;
    end
    
    % Compute absolute track axis positions
    for t = 1:len
        angle = S(t,6);
        orientation = P(t,3);
        alpha = orientation - angle;
        trackPos = S(t,5) * (TRACKWIDTH / 2);
          
        % Compute absolute positions for track axis
        if trackPos > 0 
            track(t,1:2) = P(t,1:2) + [abs(trackPos) * cos(alpha - 0.5*pi) abs(trackPos) * sin(alpha - 0.5*pi)];
        else
            track(t,1:2) = P(t,1:2) + [abs(trackPos) * cos(alpha + 0.5*pi) abs(trackPos) * sin(alpha + 0.5*pi)];
        end        
    end
    
%     plot(P(:,1),P(:,2),'-b', track(:,1),track(:,2),'-r');
    
    % Now from absolute positions to relative translations and rotations
    angles = zeros(size(track,1),1);
    map = zeros(size(track,1)-1,3);
    for t = 1:size(track,1)-1
        distance = norm(track(t+1,:) - track(t,:));
%         rotation1 = (P(t+1,3) - S(t+1,6)) - (P(t,3) - S(t,6));
        d = track(t+1,:) - track(t,:);
        
        angles(t+1) = atan2(d(2),d(1));
        rotation = angles(t+1) - angles(t);

        map(t,:) = [S(t,4) distance rotation];
    end
    
    % Close the loop
    map(end,2) = trackLength - map(end,1) + map(1,1);
    
%     % Reconstruct track from map, first point is given
%     trackPrime = zeros(size(map,1)+1,3);
%     trackPrime(1,:) = [.0007 -2.0947 0];
%     
%     angle = 0;
%     for t = 1:size(map,1)
%         angle = angle + map(t,3);
%         R = [cos(angle) -sin(angle);sin(angle) cos(angle)];
%         trackPrime(t+1,1:2) = trackPrime(t,1:2)' + R * [map(t,2); 0];
%     end
%     disp('test');
end

