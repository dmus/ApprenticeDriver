classdef ScanPlotter < handle
    %SCANPLOTTER For plotting consecutive scans.
    %   Plotting current and reference scans in Cartesian and in polar
    %   coordinate systems.
    
    properties
        C   % Current scan
        R   % Reference scan
    end
    
    methods
        function obj = ScanPlotter(C,R)
            % Set object properties
            obj.C = C;
            obj.R = R;
        end
        
        function plotCartesian(this, rotation)
            if nargin > 1
                Rot = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
            else
                Rot = eye(2);
            end

            C = this.C;
            R = this.R;
            
            % First plot the reference position and the current position
            figure;
            plot(0,0,'dm', C.position(1),C.position(2),'dc');
            hold on;
    
            % Now the edges for the reference scan, first the right
            first = R.segments(1,1);
            last = R.segments(1,2);
            plot(R.points(first:last,1),R.points(first:last,2),'+r');
    
            % Interpolate to get nice curve
            xx = R.points(first,1):.01:R.points(last,1);
            yy = spline(R.points(first:last,1),R.points(first:last,2),xx);
            plot(xx,yy,'-r');
    
            % Same for left edge of reference scan
            first = R.segments(2,1);
            last = R.segments(2,2);
            plot(R.points(first:last,1),R.points(first:last,2),'+r');

            % Interpolate to get nice curve
            xx = R.points(last,1):.01:R.points(first,1);
            yy = spline(R.points(first:last,1),R.points(first:last,2),xx);
            plot(xx,yy,'-r');

            % Translate and rotate current scan
            currentScan = bsxfun(@plus,C.points,C.position);
            currentScan = (Rot * currentScan')';

            % Now the edges for the current scan, first the right
            first = C.segments(1,1);
            last = C.segments(1,2);
            plot(currentScan(first:last,1),currentScan(first:last,2),'+b');

        %     % Interpolate to get nice curve
        %     xx = currentScan(first,1):.01:currentScan(last,1);
        %     yy = spline(currentScan(first:last,1),currentScan(first:last,2),xx);
        %     plot(xx,yy,'-b');
        %     
            % Same for left edge of current scan
            first = C.segments(2,1);
            last = C.segments(2,2);
            plot(currentScan(first:last,1),currentScan(first:last,2),'+b');

        %     % Interpolate to get nice curve
        %     xx = currentScan(last,1):.01:currentScan(first,1);
        %     yy = spline(currentScan(first:last,1),currentScan(first:last,2),xx);
        %     plot(xx,yy,'-b');

            % Now we are finished
            hold off;
        end
        
        function plotPolar(this, rotation)
            if nargin < 2
                rotation = 0;
            end
            
            figure;
            C = this.C;
            R = this.R;
            
            % First reference scan in red, first right edge
            first = R.segments(1,1);
            last = R.segments(1,2);
            plot(R.scan(first:last,2),R.scan(first:last,1),'+r');
            hold on;
            
            % Interpolate to get nice curve
            xx = R.scan(first,2):.001:R.scan(last,2);
            yy = spline(R.scan(first:last,2),R.scan(first:last,1),xx);
            plot(xx,yy,'-r');
            
            % Same for left
            first = R.segments(2,1);
            last = R.segments(2,2);
            plot(R.scan(first:last,2),R.scan(first:last,1),'+r');

            % Interpolate to get nice curve
            xx = R.scan(first,2):.001:R.scan(last,2);
            yy = spline(R.scan(first:last,2),R.scan(first:last,1),xx);
            plot(xx,yy,'-r');
            
            % Projected scan in blue
            plot(C.projection(:,2) + rotation,C.projection(:,1),'+b');
            
            % Finished
            hold off;
        end
    end
    
end

