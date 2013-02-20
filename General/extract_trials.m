function [trials, sensors] = extract_trials(filename)
%GETTRIALS Get all laps (trials) from a .mat log file of a race.
%   TRIALS = GETTRIALS(FILENAME) extracts all trials from a log file in 
%   .mat format. A new trial starts when the finish line is passed (therefore
%   the first trail will not be a full lap). A trial
%   consists of a state-action trajectory.

    T = load(filename);
    num_trials = 0;
    
    % Remove states and actions before start signal
    States = T.States(T.States(:,2) > 0,:);
    Actions = T.Actions(T.States(:,2) > 0,:);

    trackWidth = States(1,50) + States(1,68);
    sensors = States;
    
    % Separate into different trials for each lap
    % First, find starting points for each lap
    ind = find(States(:,4) < 2);
    starts = 1;
    for i = 1:length(ind)
        j = ind(i);
        if States(j - 1, 4) > States(j, 4)
            starts = [starts j];
        end
    end
    
    % Now make the trails
    for i = 1:length(starts)
        stop = size(States,1);
        if i < length(starts)
            stop = starts(i + 1) - 1;
        end

        sensors = States(starts(i):stop, :);
        actuators = Actions(starts(i):stop, :);
        
        trials{num_trials + 1}.S = compute_states(sensors);
        trials{num_trials + 1}.U = compute_actions(actuators);
        trials{num_trials + 1}.sensors = sensors;
        trials{num_trials + 1}.actuators = actuators;
        
        num_trials = num_trials + 1;
    end    
end

