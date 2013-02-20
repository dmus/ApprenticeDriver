function times = compute_discretized_times(sensors)
%COMPUTEDISCRETIZEDTIMES Finds all times relative to start signal.
%   For a new lap, times are not reset.
    times = sensors(:,2) - circshift(sensors(:,2),1);
    times(1) = 0;

    % Find resets for current lap time
    resets = find(times < 0);
    for i = 1:length(resets)
        ind = resets(i);
        times(ind) = sensors(ind,8) - sensors(ind-1,2) + sensors(ind,2);
    end

    times = cumsum(times);
end

