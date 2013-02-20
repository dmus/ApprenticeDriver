function [yawrates, uncertainty] = find_yawrates_using_splines(sensors, replaceNaN)
%FINDYAWRATE Most accurate method for yaw rate estimation.
%   Using splines and fminsearch in Cartesian coordinate system.

    if nargin < 2
        replaceNaN = false;
    end

    TRACKWIDTH = 12;

    % Compensation parameter 
    ALPHA = 0.9775;

    S(:,1) = sensors(:,47) * 1000 / 3600;
    S(:,2) = sensors(:,48) * 1000 / 3600;
    yawrates = zeros(size(S,1),1);
    uncertainty = zeros(size(yawrates));
    
    times = compute_discretized_times(sensors);
    t = 6023;%40;%536;
    %for t = 536:566
    
    indices = 1:19;
    angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);

    
    for t = 2:size(sensors,1)
        if mod(t,500) == 0
            fprintf('t:%d = %0.3f\n', t, times(t));
        end
        
        % Compute move
        dt = times(t) - times(t-1);
        translation = [.5*(S(t-1,1)+S(t,1)) .5*(S(t-1,2)+S(t,2))] .* dt;
        
        C.position = translation;
        C.orientation = 0;
        C.scan = [flipud(sensors(t, indices + 49)') flipud(angles)];
        
        % The reference scan
        R.scan = [flipud(sensors(t-1, indices + 49)') flipud(angles)];
        
        % A. Scan Preprocessing
        % First ranges are segmented, to prevent interpolation between left
        % and right track edge
        
        % Remove ranges > 30
        C.scan(C.scan(:,1) > 25,:) = [];
        R.scan(R.scan(:,1) > 35,:) = [];
        
        % TODO can this without computing points in x-y coordinates?
        C.points = [C.scan(:,1) .* cos(C.scan(:,2)) C.scan(:,1) .* sin(C.scan(:,2))];
        C.segments = segment(C.points, TRACKWIDTH - 0.5);
        
        R.points = [R.scan(:,1) .* cos(R.scan(:,2)) R.scan(:,1) .* sin(R.scan(:,2))];
        R.segments = segment(R.points, TRACKWIDTH - 0.5);
        
        yawRatesPerSegment = zeros(1,length(R.segments));
        for s = 1:length(R.segments)
            first = C.segments(s,1);
            last = C.segments(s,2);
            
            firstRef = R.segments(s,1);
            lastRef = R.segments(s,2);
        
            ppref = spline(R.points(firstRef:lastRef,1),R.points(firstRef:lastRef,2));
            
            while sum(ppref.coefs(:,1) > 1) > 0
                lastRef = lastRef - 1;
                ppref = spline(R.points(firstRef:lastRef,1),R.points(firstRef:lastRef,2));
            end
            
            % No extrapolation, because that will give very inaccurate
            % results
            if s == 2
                pointsToMatch = flipud(C.points(first:last,:));
                refPoints = flipud(R.points(firstRef:lastRef,:));
            else
                pointsToMatch = C.points(first:last,:);
                refPoints = R.points(firstRef:lastRef,:);
            end
            
            outOfRange = ((pointsToMatch(:,1) + C.position(1)) > refPoints(end,1)) | ((pointsToMatch(:,1) + C.position(1)) < refPoints(1,1));
            pointsToMatch(outOfRange,:) = [];
            f = @(x) scan_error(pointsToMatch,ppref,C.position,x);
            yawRatesPerSegment(s) = fminsearch(f, 0);
        end
        difference = yawRatesPerSegment(2) - yawRatesPerSegment(1);
        uncertainty(t) = difference;
        yawrate = mean(yawRatesPerSegment);
        
        %yawrate = yawrate * alpha;
        
        % FIX FOR ERROR IN RACETRACK
        if sensors(t,4) < sensors(t-1,4)-100
            yawrate = NaN; % Wrong sensor information
            uncertainty(t) = NaN;
        end

        if abs(difference) > 0.01 && ~isnan(yawrate)
            fprintf('Warning: high uncertainty at %d\n', t);
        end
        
        yawrates(t) = (yawrate * ALPHA) / dt;
    end
    
    if replaceNaN
        nans = find(isnan(yawrates));
        % Interpolate to get an estimate
        yawrates(nans) = (yawrates(nans - 1) + yawrates(nans + 1)) / 2;
    end
end

