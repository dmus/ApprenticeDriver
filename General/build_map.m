function T = build_map(S, sensors)
%BUILDMAP Summary of this function goes here
%   T is a tuple consisting of (distance along track axis, distance to next
%   point, rotation to next point).

    trackWidth = 12;
    
    nPoints = size(S,1);
    
    % Track will contain all absolute positions of the track axis
    track = zeros(nPoints,3);
    left = zeros(nPoints,2);
    right = zeros(nPoints,2);
    
    from = 1;%size(Track,1)-300;
    to = nPoints;
    %for t = 1:size(Track,1)
    for t = from:to
        angle = S(t,6);
        orientation = S(t,3);
        alpha = orientation - angle;
        trackPos = S(t,5) * (trackWidth / 2);
        
%         horizon = S(t,1:2) + [abs(trackPos) * cos(orientation) abs(trackPos) * sin(orientation)];
%         temp = horizon' - S(t,1:2)';
        
        if trackPos > 0 
            track(t,1:2) = S(t,1:2) + [abs(trackPos) * cos(alpha - 0.5*pi) abs(trackPos) * sin(alpha - 0.5*pi)];
        else
            track(t,1:2) = S(t,1:2) + [abs(trackPos) * cos(alpha + 0.5*pi) abs(trackPos) * sin(alpha + 0.5*pi)];
        end
        
        
        
%         R = [cos(beta) -sin(beta); sin(beta) cos(beta)];
%         Track(t,1:2) = (R * temp)' + S(t,1:2);
%         
        %trackPosLeft = 0.5 * trackWidth - trackPos;
        %trackPosRight = -0.5 * trackWidth - trackPos;
        
        left(t,1:2) = track(t,1:2) + [0.5*trackWidth * cos(alpha + 0.5*pi) 0.5*trackWidth * sin(alpha + 0.5*pi)];
        right(t,1:2) = track(t,1:2) + [0.5*trackWidth * cos(alpha - 0.5*pi) 0.5*trackWidth * sin(alpha - 0.5*pi)];
        
        %Track(t,1:2) = S(t,1:2) - [trackPos * sin(alpha) trackPos * cos(alpha)];
        %Left(t,1:2) = S(t,1:2) + [trackPosLeft * sin(alpha) trackPosLeft * cos(alpha)];
        %Right(t,1:2) = S(t,1:2) + [trackPosRight * sin(alpha) trackPosRight * cos(alpha)];
    end
    
    
    
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
    
    T = zeros(nPoints-1,3);
    for t = 1:nPoints-1
        distance = sqrt(sum((track(t+1,:) - track(t,:)) .^ 2));
        rotation = (S(t+1,3) - sensors(t+1,1)) - (S(t,3) - sensors(t,1));
        T(t,:) = [sensors(t,4) distance rotation];
    end
    
end

