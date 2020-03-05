function write_graph(pathToWrite, conf, vertex, edges)

nPoints = size(conf, 1);

% conf
fId = fopen([pathToWrite '_conf'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f\n', conf(k, 1), conf(k, 2), conf(k, 3));
end
fclose(fId);

% vertex
fId = fopen([pathToWrite '_vertex'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f ', vertex(k, 1), vertex(k, 2), vertex(k, 3));
    pointsInSight = vertex(k, 4:end);
    fprintf(fId, strrep(strrep(num2str(pointsInSight(~isnan(pointsInSight))), '   ', ' '), '  ', ' '));
    fprintf(fId, '\n');
end
fclose(fId);

% edges
fId = fopen([pathToWrite '_edge'], 'w');
for k = 1:size(edges, 1)
    fprintf(fId, '%d %d %d %d %d %d %f\n', edges(k, :));
end
fclose(fId);
