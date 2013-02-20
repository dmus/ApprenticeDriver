run('../addpaths');

trajectory = build_trajectory('Wheel-2_MrRacer.mat', 3, 300);
plotter = TrajectoryPlotter(trajectory);
plotter.plot(1,200);