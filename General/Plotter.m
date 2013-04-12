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
        
        function addTrial(this, trial)
            TRACKWIDTH = 12;
            s1 = trial.S(1,:)';
            m2 = find(this.map(:,1) > s1(4),1);
            m1 = m2 - 1;
            
            u = (s1(4) - this.map(m1,1)) / (this.map(m2,1) - this.map(m1,1));
            
            p1 = this.X(m1,1:2)';
            p2 = this.X(m2,1:2)';
            
            % p is the starting point for trial
            p = p1 + u * (p2 - p1);
            angle = this.X(m1,3) + u * this.map(m1,3);
            
            lateral = s1(5) * 0.5 * TRACKWIDTH;
            Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            p = p + Rot * [0; lateral];
            
            S = trial.S;
            X = zeros(size(S,1),2);
            X(1,:) = p';
            
            for t = 1:size(S,1)-1
                dt = trial.T(t+1) - trial.T(t);
                move = 0.5 * (S(t,1:2) + S(t+1,1:2))' * dt;
                Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
                X(t+1,:) = X(t,:) + (Rot * move)';
                angle = angle + S(t+1,3) * dt;
            end
            figure(this.h);
            plot(X(:,1),X(:,2), 'b');
        end
    end
    
end

