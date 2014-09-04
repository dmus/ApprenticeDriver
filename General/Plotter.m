classdef Plotter < handle
    %PLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        map     % Map
        trials  % Agent trials
        h       % Handle
        X       % Absolute map coordinates
    end
    
    methods
        function obj = Plotter(map)
            obj.map = map;
            [X, h] = show_map(map, 1, 2);
            obj.X = X;
            obj.h = h;
        end
        
        function p = findPoint(this, s)
            TRACKWIDTH = 12;
            
            s1 = s;
            
            % Find first track axis point after s1
            m2 = find(this.map(:,1) > s1(4),1);
            
            % And the one before s1
            m1 = m2 - 1;
            
            % Part of track between m1 and m2 along track axis covered
            u = (s1(4) - this.map(m1,1)) / (this.map(m2,1) - this.map(m1,1));
            
            % Two track axis points in absolute coordinates
            p1 = this.X(m1,1:2)';
            p2 = this.X(m2,1:2)';
            
            % p is the starting point for trial
            p = p1 + u * (p2 - p1);
            angle = this.X(m1,3) + u * this.map(m1,3);
            
            lateral = s1(5) * 0.5 * TRACKWIDTH;
            Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            p = p + Rot * [0; lateral];
            % Now we have one point on the map
        end
        
        function addTrial(this, trial, color)
            % Starting state
            s1 = trial.S(1,:)';
            
            p = this.findPoint(s1);
            
            S = trial.S;
            X = zeros(size(S,1),2);
            X(1,:) = p';
            
            for t = 1:size(S,1)
                s1 = trial.S(t,:)';
                p = this.findPoint(s1);
                X(t,:) = p';
            end
%             for t = 1:size(S,1)-1
%                 dt = trial.T(t+1) - trial.T(t);
%                 move = 0.5 * (S(t,1:2) + S(t+1,1:2))' * dt;
%                 Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
%                 X(t+1,:) = X(t,:) + (Rot * move)';
%                 angle = angle + S(t+1,3) * dt;
%             end
            figure(this.h);
            plot(X(:,1),X(:,2), color, 'LineWidth', 2);
        end
    end
    
end

