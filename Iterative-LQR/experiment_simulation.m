% Test controller in our simulator, without the real simulator
addpath('../Data', '../General', '../Yawrate-Estimation', '../Acceleration-Model');

g = @g_constantspeed;
H = 100;
reference = build_trajectory('Wheel-2_MrRacer.mat', 3, H);
map = [];
driver = Controller(H, reference, model, map);

s = zeros(22,1);
%s([1 9 17]) = 5;
time = 0;
dt = 0.02;

cost = 0;
for t = 1:H-1
    u = driver.controlFromState(s, time);
    cost = cost + g(s) + h(u);
    u(2) = 0;
    if u(1) < 0
        disp('Ho');
    end
    
    time = time + dt;
    s = f(s, u, dt, model, map);
end
cost = cost + g(s);
