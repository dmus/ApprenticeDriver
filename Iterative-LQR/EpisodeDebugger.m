classdef EpisodeDebugger < handle
    %EPISODEPLOTTER For completely debugging an episode.
    %   Plotting and debugging utilities
    
    properties
        driver
        S
        U
        H
        T
    end
    
    methods
        function obj = EpisodeDebugger(driver)
            obj.driver = driver;
            obj.S = driver.history.S;
            obj.U = driver.history.U;
            obj.T = driver.history.T;
            obj.H = length(obj.T);
        end
        
        function plotInputs(this)
            H = this.H;
            figure;
            plot(1:H-1,this.episode.U(:,1),'-r',1:H-1,this.episode.U(:,2),'-b');
            hold on;
            plot(1:H-1,this.reference.U(:,1),'--r',1:H-1,this.reference.U(:,2),'--b');
            legend('Acceleration','Steering','ref','ref');
            hold off;
        end
        
        function plotStates(this)
            figure;
%             subplot(2,2,1);
%             plot(1:this.H,this.episode.S(:,5),'-r', 1:this.H,this.reference.S(:,5),'--r');
%             hold on;
%             plot(1:this.H,this.episode.S(:,6),'-b', 1:this.H,this.reference.S(:,6),'--b');
%             legend('trackPos','ref','angle','ref');
%             hold off;
%             
%             subplot(2,2,2);
%             plot(1:this.H,this.episode.S(:,4),'-r', 1:this.H,this.reference.S(:,4),'--r');
%             legend('distFromStart','ref');
            
            % Plot velocities
            subplot(2,2,3);
            hold on;
            plot(1:this.H,this.episode.S(:,2),'-b', 1:this.H,this.reference.S(:,2),'--b');
            %plot(1:this.H,this.episode.S(:,3),'-g', 1:this.H,this.reference.S(:,3),'--g');
            %legend('speedY','ref','angularSpeed','ref');
            hold off;
            
            subplot(2,2,4);
            plot(1:this.H,this.episode.S(:,1),'-r', 1:this.H,this.reference.S(:,1),'--r');
            legend('speedX','ref');
        end
        
        function plotCosts(this)
            figure;
            % Compute cost for episode
            cost = zeros(1,this.H);
            refCost = zeros(1,this.H);
            
            prevCost = 0;
            prevRefCost = 0;
            for t = 1:this.H-1
                cost(t) = prevCost + g(this.episode.S(t,:)) + h(this.episode.U(t,:)');
                refCost(t) = prevRefCost + g(this.reference.S(t,:)) + h(this.reference.U(t,:)');
                
                prevCost = cost(t);
                prevRefCost = refCost(t);
            end
            
            cost(this.H) = cost(this.H-1) + g(this.episode.S(this.H,:));
            refCost(this.H) = refCost(this.H-1) + g(this.reference.S(this.H,:));
            
            plot(1:this.H,cost,'-r',1:this.H,refCost,'--r');
            legend('Cost','ref');
        end
        
        function testDynamics(this)
            v_prev = [0;0];
            for t = 1:this.H-1
                dt = this.episode.T(t+1) - this.episode.T(t);
                
                z_t = this.episode.S(t,:)' - this.reference.S(t,:)';
                v_t = this.episode.U(t,:)' - this.reference.U(t,:)';
                
                zprime = [z_t; 1; v_prev; 1];
                dv = v_t - v_prev;
                
                zprime_next = this.driver.A{t} * zprime + this.driver.B{t} * dv;
                
                s_tplus1 = zprime_next(1:3) + this.reference.S(t,:)';
                s_next = f(this.episode.S(t,:)', this.episode.U(t,:)', dt, this.driver.Map, this.driver.model);
                target = this.episode.S(t+1,:)';
                
                v_prev = v_t;
            end
        end
        
        function testCosts(this)
            for t = 1:this.H-1
                % Test Q matrix
                s = this.episode.S(t,:)';
                z = s - this.reference.S(t,:)';
                cost = g(s);
                penalty = norm(z)^2;
                
                alpha = 0.99;
                cost = (1-alpha) * cost + alpha * penalty;
                
                approximation = z' * this.driver.Q{t} * z;
                diff = cost - approximation;
                
                % Same for R matrix
                u = this.episode.U(t,:)';
                v = u - this.reference.U(t,:)';
                cost = h(u);
                penalty = norm(v)^2;
                
                alpha = 0.99;
                cost = (1-alpha) * cost + alpha * penalty;
                
                approximation = v' * this.driver.R{t} * v;
                diff = cost - approximation;
            end
        end
        
        function simulate(this)
            
            for t = 1:this.H-1
                dt = this.T(t+1) - this.T(t);
                
                s = this.S(t,:)';
                u = this.driver.controlFromState(s);
                
                z_t = [s - this.reference.S(t,:)'; 1; v_prev; 1];
                dv = this.driver.K{t} * z_t;
                v_t = v_prev + dv;
                
                z_next = this.driver.A{t} * z_t + this.driver.B{t} * dv;
                
                s_next = z_next(1:3) + this.reference.S(t,1:3)';
                target = this.episode.S(t+1,1:3)';
                pred = f(s,u,dt,this.driver.Map,this.driver.model);
                
                % Choose dv to minimize result
                result = z_t' * this.driver.Q{t} * z_t + dv' * this.driver.R{t} * dv + z_next' * this.driver.P{t+1} * z_next;
                fun = @(x) z_t' * this.driver.Q{t} * z_t + x' * this.driver.R{t} * x + (this.driver.A{t} * z_t + this.driver.B{t} * x)' * this.driver.P{t+1} * (this.driver.A{t} * z_t + this.driver.B{t} * x);
                v = fminsearch(fun,[0 0]');
                
                this.plotModel(t, s, z_t);
            end
        end
        
%         function ret = f(this, s, u, dt, el)
%             s_next = f(s, u, dt, this.driver.Map, this.driver.model);
%             ret = s_next(el);
%         end
        
        % Plot possible steering commands vs yawrates
        function plotModel(this, t, s, z)
            dt = this.episode.T(t+1) - this.episode.T(t);
            
            u_a = 1;
            u_s = -1:.001:1;
            
            % For our nonlinear model
            fun = @(u) f(s, u, dt, this.driver.Map, this.driver.model);
            
            
            yawrate = zeros(size(u_s));
            ydot = zeros(size(u_s));
            
            for i = 1:length(u_s)
                s_next = fun([u_a;u_s(i)]);
                ydot(i) = s_next(2);
                yawrate(i) = s_next(3);
            end
            
            disp(t);
            disp('Nonlinear');
            [~, ind] = sort(abs(yawrate));
            u_best = u_s(ind(1))
            % fzero();
            
            figure(1);
            plot(u_s,yawrate,'-r',u_s,ydot,'-b');
            
            % For our linear approximations
            v_prev = z(5:6);
            fun = @(u_s) f(s,[1;u_s],dt,this.driver.Map,this.driver.model);
            
            yawrate = zeros(size(u_s));
            ydot = zeros(size(u_s));
            
            for i = 1:length(u_s)
                u = [1; u_s(i)];
                v = u - this.reference.U(t,:)';
                
                dv = v - v_prev;
                
                s_next = this.next(t, z, dv);
                ydot(i) = s_next(2);
                yawrate(i) = s_next(3);
            end
            
            disp('Linear');
            [~, ind] = sort(abs(yawrate));
            u_best = u_s(ind(1))
            
            figure(2);
            plot(u_s,yawrate,'-r',u_s,ydot,'-b');
        end
        
        function [s_next, u] = next(this, t, z_t, dv)
            z_next = this.driver.A{t} * z_t + this.driver.B{t} * dv;
            v = z_next(5:6);
            u = v + this.reference.U(t,:)';
            s_next = z_next(1:3) + this.reference.S(t+1,:)';
        end
    end
    
end

