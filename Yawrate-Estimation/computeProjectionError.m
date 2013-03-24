function [error] = computeProjectionError(LandmarksLeft, LandmarksRight, move, PointsLeft, PointsRight, angle)
%COMPUTEPROJECTIONERROR Summary of this function goes here
%   Detailed explanation goes here
    error = 0;
    
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    
    % Rotate new points around origin and add translation
    PointsLeft = bsxfun(@plus, R*PointsLeft', move')';
    PointsRight = bsxfun(@plus, R*PointsRight', move')';
    
    % Range finders at the left side
    index = 1;
    for i = 1:size(PointsLeft,1)
        Point = PointsLeft(i,:)';

        while true
            dx = LandmarksLeft(index+1,1) - LandmarksLeft(index,1);
            dy = LandmarksLeft(index+1,2) - LandmarksLeft(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - LandmarksLeft(index,1);
            Ry = Point(2) - LandmarksLeft(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
                if index >= size(LandmarksLeft,1)
                    break;
                end
            else
                break;
            end
        end
        
        if index >= size(LandmarksLeft,1)
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        % CTE is relative, now compute absolute error
        cte = cte * norm([dx;dy]);
        
        error = error + cte^2;
    end
    
    % Same for right side
    index = 1;
    for i = 1:size(PointsRight,1)
        Point = PointsRight(i,:)';
        
        while true
            dx = LandmarksRight(index+1,1) - LandmarksRight(index,1);
            dy = LandmarksRight(index+1,2) - LandmarksRight(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - LandmarksRight(index,1);
            Ry = Point(2) - LandmarksRight(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
                if index >= size(LandmarksRight,1)
                    break;
                end
            else
                break;
            end
        end
        
        if index >= size(LandmarksRight,1)
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        % CTE is relative, now compute absolute error
        cte = cte * norm([dx;dy]);
        
        
        error = error + cte^2;
    end
end

