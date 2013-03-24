function [PointsLeft, PointsRight] = groupRangeFinders(Points, threshold)
%GROUPRANGEFINDERS Summary of this function goes here
%   Detailed explanation goes here
    distances = Points(2:end,:) - Points(1:end-1,:);
    distances = sqrt(sum(distances .^2, 2));
    
    gaps = find(distances >= threshold);
    gaps = [0; gaps; length(Points)];
    groups = zeros(length(gaps)-1,2);
    for i = 2:length(gaps)
        last = gaps(i);
        first = gaps(i-1) + 1;
            
        groups(i-1,:) = [first last];
    end
        
    groupSize = groups(:,2) - groups(:,1) + 1;
    [~,i] = sort(groupSize, 1, 'descend');
    Landmarks1 = Points(groups(i(1),1):groups(i(1),2),:);
    Landmarks2 = Points(groups(i(2),1):groups(i(2),2),:);
        
    if Landmarks1(1,1) < Landmarks1(end,1)
        PointsLeft = Landmarks1;
        PointsRight = flipud(Landmarks2);
    else
        PointsRight = flipud(Landmarks1);
        PointsLeft = Landmarks2;
    end
end

