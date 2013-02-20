classdef TrajectoryPlotter
    %TRAJECTORYPLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        S
        U
        T
    end
    
    methods
        function obj = TrajectoryPlotter(trajectory)
            obj.S = trajectory.S;
            obj.U = trajectory.U;
            obj.T = trajectory.T;
        end
        
        function plot(this, from, to)
            x = from:to;
            plot(x, this.U(from:to,2), '--b', x, this.S(from:to,3), '-r');
            h = legend('$u_s$', '$\dot{\omega}$');
            set(h,'Interpreter','latex');
            set(h,'FontSize',14);
        end
    end
    
end

