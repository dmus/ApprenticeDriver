addpath('../Data', '../General', '../Yawrate-Estimation');

[trials] = extract_trials('Wheel-2_SimpleDriver.mat');
times = compute_discretized_times(trials{2}.sensors);

map = build_map(trials{2}.S, times);
show_map(map);