function points = read_bridge_model()

fId = fopen(fullfile('..', 'data', 'bridge', 'bridge_small.obj'));
line = fgetl(fId);
points = [];
while ~contains(line, 'End of File')
    if ~isempty(line)
        if line(1) == 'v' && line(2) == ' '
            point = str2num(line(3:end));
            points = [points; point];
        end
    end
    line = fgetl(fId);
end
fclose(fId);