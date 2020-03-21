function points = read_bridge_model(type)

if exist('type', 'var') && strcmp(type, 'big')
    fId = fopen(fullfile('..', 'data', 'bridge', 'bridge.obj'));
else
    type = 'small';
    fId = fopen(fullfile('..', 'data', 'bridge', 'bridge_small.obj'));
end
line = fgetl(fId);
points = [];
while (strcmp(type, 'small') && ~contains(line, 'End of File')) || (strcmp(type, 'big') && all(line~=-1))
    if ~isempty(line)
        if line(1) == 'v' && line(2) == ' '
            point = str2num(line(3:end));
            points = [points; point];
        end
    end
    line = fgetl(fId);
end
fclose(fId);