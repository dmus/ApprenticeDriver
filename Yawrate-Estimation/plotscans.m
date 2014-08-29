%% Script to get a nice plot to demonstrate the working of the yawrate estimation
t = 2743;
TRACKWIDTH = 12;

S = load('Wheel-2_MrRacer.mat');
% Remove states and actions before start signal
sensors = S.States(S.States(:,2) > 0,:);

S = trajectory.S;

% Compute move
indices = 1:19;
angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);
dt = trajectory.T(t) - trajectory.T(t-1);
translation = [.5*(S(t-1,1)+S(t,1)) .5*(S(t-1,2)+S(t,2))] .* dt;
        
C.position = translation;
C.orientation = 0;
C.scan = [flipud(sensors(t, indices + 49)') flipud(angles)];
        
% The reference scan
R.scan = [flipud(sensors(t-1, indices + 49)') flipud(angles)];

C.points = [C.scan(:,1) .* cos(C.scan(:,2)) C.scan(:,1) .* sin(C.scan(:,2))];
C.segments = segment(C.points, TRACKWIDTH - 0.5);
        
R.points = [R.scan(:,1) .* cos(R.scan(:,2)) R.scan(:,1) .* sin(R.scan(:,2))];
R.segments = segment(R.points, TRACKWIDTH - 0.5);

plotter = ScanPlotter(C, R);
plotter.plotCartesian();
plotter.plotCartesian(0.05);