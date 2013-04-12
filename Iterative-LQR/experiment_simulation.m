% Test controller in our simulator, without the real simulator
g = @g_constantspeed;
H = 50;
% reference = build_trajectory('Wheel-2_MrRacer.mat', 10, 1000);
%reference.S(:,[4:8:end]) = 0;

R = load('reference.mat');
load('model.mat');
load('trials.mat');
times = compute_discretized_times(trials{2}.sensors);

map = build_map(trials{2}.S, times);

window = 1:250;
H = length(window);
reference.S = R.reference.S(window,:);
reference.U = R.reference.U(window,:);
reference.T = R.reference.T(window,:);

driver = Controller(length(window), reference, model, map);

% Initial state
% s = zeros(10*8-2,1);


%s([1 9 17]) = 5;
time = 0;
dt = 0.02;

p = Plotter(map);

for iterations = 1:10
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
    
    % Plot
    p.addTrial(driver.history);
    
    driver.reset();
end

