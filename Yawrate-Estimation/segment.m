function segments = segment(Points, threshold)
%SEGMENT Summary of this function goes here
%   Detailed explanation goes here

    % Compute squared distances between consecutive points
    distances = Points(2:end,:) - Points(1:end-1,:);
    distances = sqrt(sum(distances .^2, 2));
    
    % Form segments
    gaps = find(distances >= threshold);
    gaps = [0; gaps; length(Points)];
    segments = zeros(length(gaps)-1,2);
    for i = 2:length(gaps)
        last = gaps(i);
        first = gaps(i-1) + 1;
            
        segments(i-1,:) = [first last];
    end
    
    % Find two biggest segments
    segmentSize = segments(:,2) - segments(:,1) + 1;
    [~,i] = sort(segmentSize, 1, 'descend');
    
    % Return only two segments
    segments = sort([segments(i(1),:); segments(i(2),:)]);
end

