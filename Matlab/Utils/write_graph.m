function write_graph(pathToWrite, G)

nVars = length(G.Nodes.Properties.VariableNames);
confVarIdx = [];
for i = 1:nVars
    for j = 1:nVars
        if strcmp(G.Nodes.Properties.VariableNames{j}, ['x' num2str(i)])
            confVarIdx = [confVarIdx j];
        end
    end
end
conf = table2array(G.Nodes(:, confVarIdx));

nPoints = size(conf, 1);

% conf
fId = fopen([pathToWrite '_conf'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f\n', k-1, conf(k, 1), conf(k, 2));
end
fclose(fId);

% vertex
fId = fopen([pathToWrite '_vertex'], 'w');
for k = 1:nPoints
    fprintf(fId, '%d %f %f ', k-1, G.Nodes.time_vis(k), G.Nodes.time_build(k));
    pointsInSight = G.Nodes.vis{k};
    fprintf(fId, strrep(strrep(num2str(pointsInSight(~isnan(pointsInSight))), '   ', ' '), '  ', ' '));
    fprintf(fId, '\n');
end
fclose(fId);

% edges
fId = fopen([pathToWrite '_edge'], 'w');
for k = 1:size(G.Edges, 1)
    fprintf(fId, '%d %d %d %d %d %d %f\n', table2array(G.Edges(k, 1:end-1)));
end
fclose(fId);
end