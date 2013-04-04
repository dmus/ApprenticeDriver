% trials = extract_trials('Wheel-2_MrRacer.mat');
% save('trials.mat', 'trials');
load('trials.mat');
map = build_map(trials{2}.S);

H = 50;
reference = build_trajectory('Wheel-2_MrRacer.mat', 10, H);

load('model.mat');
driver = Controller(H, reference, model, map);

load('trial.mat');

for step = 1:size(States,1)
    driver.control(States(step,:)');
end