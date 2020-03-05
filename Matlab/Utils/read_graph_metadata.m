function [obstacles, inspectionPoints, params] = read_graph_metadata(base_name)

if ~exist([base_name, '_obstacles'], 'file')
    [obstacles, inspectionPoints, params] = deal([]);
    return
end

data = importdata([base_name, '_obstacles']);
nObstacles = max(data(:,1))+1;
obstacles = cell(nObstacles,1);
for k = 1:nObstacles
    obstacles{k} = data(data(:,1)==k-1,2:3);
end

data = importdata([base_name, '_inspectionPoints']);
inspectionPoints = data(:,2:3);

data = importdata([base_name, '_params']);
params = [];
nParams = numel(data.textdata);
for k = 1:nParams
    params.(data.textdata{k}) = data.data(k);
end