% [trials] = extract_trials('Wheel-2_SimpleDriver.mat');
load('trials.mat');
times = compute_discretized_times(trials{2}.sensors);

map = build_map(trials{2}.S, times);
hold on;
show_map(map);