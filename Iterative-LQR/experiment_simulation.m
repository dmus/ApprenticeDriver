% Test controller in our simulator, without the real simulator
g = @g_constantspeed;
H = 20;
reference = build_trajectory('Wheel-2_MrRacer.mat', 10, H);
%reference.S(:,[4:8:end]) = 0;
load('model.mat');
load('trials.mat');
times = compute_discretized_times(trials{2}.sensors);

map = build_map(trials{2}.S, times);

driver = Controller(H, reference, model, map);

% Initial state
% s = zeros(10*8-2,1);


%s([1 9 17]) = 5;
time = 0;
dt = 0.02;


for iterations = 1:5
    s = reference.S(1,:)';
    costS = 0;
    costU = 0;
    for t = 1:H
        u = driver.controlFromState(s, time);
        costS = costS + g(s);
        costU = costU + h(u);

        time = time + dt;
        s = f(s, u, dt, model, map);
    end
    costS = costS + g(s);
    fprintf('Cost states %f Cost inputs %f\n', costS, costU);
    driver.reset();
end

