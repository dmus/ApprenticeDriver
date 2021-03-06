function convert(filename)
%CONVERT Convert log of race to a states and action matrices saved in a
%file
%   CONVERT(FILENAME) assumes FILENAME is without extension, assumed extension is '.log'. File is read,
%   resulting matrices are saved to FILENAME.mat
    fid = fopen(strcat(filename, '.log'), 'r');

    States = zeros(0, 79);
    Actions = zeros(0, 7); 

    % Read first lines
    line = fgetl(fid);
    while isempty(strfind(line, '***identified***'))
        line = fgetl(fid);
    end
    
    line = fgetl(fid);
    while ~strcmp(line, 'Received: ***shutdown***') && ~strcmp(line, 'Received: ***restart***')
        mode = sscanf(line, '%s', 1);

        if strcmp(mode, 'Received:')
            s = sscanf(line, 'Received: (angle %f)(curLapTime %f)(damage %f)(distFromStart %f)(distRaced %f)(fuel %f)(gear %f)(lastLapTime %f)(opponents %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(racePos %f)(rpm %f)(speedX %f)(speedY %f)(speedZ %f)(track %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(trackPos %f)(wheelSpinVel %f %f %f %f)(z %f)(focus %f %f %f %f %f)');
            States = [States; s'];
        elseif strcmp(mode, 'Sending:')
            a = sscanf(line, 'Sending: (accel %f) (brake %f) (clutch %f) (gear %f) (steer %f) (meta %f) (focus %f)');
            Actions = [Actions; a'];
        end

        line = fgetl(fid);
    end

    fclose(fid);

    save(strcat(filename, '.mat'), 'States', 'Actions');
end

