function error = scan_error(points,ppref,translation,rotation)
%ERROR Summary of this function goes here
%   Detailed explanation goes here
    R = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
    points = bsxfun(@plus, R*points', translation')';
    
    error = norm(ppval(ppref, points(:,1)) - points(:,2));    
end

