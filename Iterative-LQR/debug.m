H = 50;
reference = build_trajectory('Wheel-2_MrRacer.mat', 10, H);
map = [];
load('model.mat');
driver = Controller(H, reference, model, map);

load('trial.mat');

for step = 1:size(States,1)
    driver.control(States(step,:)');
end