% Experiment in real simulator

% Start the simulator
oldDir = cd('C:/Program Files (x86)/torcs');
system('wtorcs.exe -t 100000000 &');
cd(oldDir);

% Now start the client
H = 250;
reference = build_trajectory('Wheel-2_MrRacer.mat', 10, H);
%map = [];
load('map.mat');
load('model.mat');
driver = Controller(H, reference, model, map);

% Run client for one episode
client = Client(driver, H, 1);
output = evalc('client.run()');

% Write to file
disp('Writing output to file...');
fid = fopen('trial.log', 'wt');
fprintf(fid, '%s\n', output);
fclose(fid);

% Process and visualize
disp('Converting log file to matrices...');
convert('trial');
disp('Ready');