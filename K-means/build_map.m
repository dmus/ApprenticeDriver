function map = build_map(S, times)
%BUILDMAP Build a map from sensor information.
%   MAP consists of a series of tuples of the form (distanceAlongTrackAxis,
%   distance, rotation). Each tuple described with distance and rotation
%   the rotation and distance to the next point on the track axis.

    TRACKWIDTH = 12;
    
    len = size(S,1);
    S(:,7:9) = zeros(len,3);
    
    track = zeros(len,2);
    left = zeros(len,2);
    right = zeros(len,2);
    
    % Compute absolute positions, first position is fixed at (0,0)
    for t = 2:len
        dt = times(t) - times(t-1);
        velocity = 0.5 * (S(t-1,1:2) + S(t,1:2));
        
        % Compute absolute position and orientation
        rotation = S(t-1,9);
        R = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
        
        S(t,7:8) = S(t-1,7:8) + (R * (velocity .* dt)')';
        S(t,9) = S(t-1,9) + S(t,3) * dt;
        
        angle = S(t,6);
        orientation = S(t,9);
        alpha = orientation - angle;
        trackPos = S(t,5) * (TRACKWIDTH / 2);
        
%         horizon = S(t,1:2) + [abs(trackPos) * cos(orientation) abs(trackPos) * sin(orientation)];
%         temp = horizon' - S(t,1:2)';
        
        % Compute absolute positions for track axis
        if trackPos > 0 
            track(t,1:2) = S(t,7:8) + [abs(trackPos) * cos(alpha - 0.5*pi) abs(trackPos) * sin(alpha - 0.5*pi)];
        else
            track(t,1:2) = S(t,7:8) + [abs(trackPos) * cos(alpha + 0.5*pi) abs(trackPos) * sin(alpha + 0.5*pi)];
        end
        
        
        
%         R = [cos(beta) -sin(beta); sin(beta) cos(beta)];
%         Track(t,1:2) = (R * temp)' + S(t,1:2);
%         
        %trackPosLeft = 0.5 * trackWidth - trackPos;
        %trackPosRight = -0.5 * trackWidth - trackPos;
        
        left(t,1:2) = track(t,1:2) + [0.5*TRACKWIDTH * cos(alpha + 0.5*pi) 0.5*TRACKWIDTH * sin(alpha + 0.5*pi)];
        right(t,1:2) = track(t,1:2) + [0.5*TRACKWIDTH * cos(alpha - 0.5*pi) 0.5*TRACKWIDTH * sin(alpha - 0.5*pi)];
        
        %Track(t,1:2) = S(t,1:2) - [trackPos * sin(alpha) trackPos * cos(alpha)];
        %Left(t,1:2) = S(t,1:2) + [trackPosLeft * sin(alpha) trackPosLeft * cos(alpha)];
        %Right(t,1:2) = S(t,1:2) + [trackPosRight * sin(alpha) trackPosRight * cos(alpha)];
    end
    
%     plot(track(:,1),track(:,2),'-b');
    
%     plotTrajectory(Track, 'b');
%     hold on;
% %     plotTrajectory(Left, 'black');
% %     plotTrajectory(Right, 'black');
%     plotTrajectory(S, 'r');
%     
%     plotTrajectory(Track(from:to,:), 'b');
%     hold on;
%     
%     %plotTrajectory(Left(from:to,:), 'black');
%     %plotTrajectory(Right(from:to,:), 'black');
%     
%     plotTrajectory(S(from:to,:), 'r');
%     plot([Left(from:to,1) Right(from:to,1)]', [Left(from:to,2) Right(from:to,2)]','b-')
%     plot([Track(from:to,1) S(from:to,1)]', [Track(from:to,2) S(from:to,2)]', 'b-');
%     
%     hold off;
    
    map = zeros(size(track,1)-1,3);
    for t = 1:size(track,1)-1
        distance = norm(track(t+1,:) - track(t,:));
        %distance = sqrt(sum((track(t+1,:) - track(t,:)) .^ 2));
        rotation = (S(t+1,9) - S(t+1,6)) - (S(t,9) - S(t,6));
        map(t,:) = [S(t,4) distance rotation];
    end
    
end

