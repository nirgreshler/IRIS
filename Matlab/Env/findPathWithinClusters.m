clear
close all
clc

env_name = 'syn_9rooms';
addpath(genpath(pwd))

base_name = fullfile(pwd, 'Graphs');
username = getenv('USERNAME');
wsl_path = ['/home/' username '/Project/IRIS'];
search_path = [wsl_path, '/debug/app/search_graph'];
base_name_in_wsl = ['/mnt/' lower(strrep(strrep(base_name,':',''),'\','/'))];

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));
M = Edges2M(edges(:,[1 2 7]));
M(M>0)=1;
[clustersKmeans, clustersSpectral] = ClusterPoints(conf(:,2:3), M);
%%
clusterNum = 1;
pointsInCluster = clustersSpectral == clusterNum;
points = conf(pointsInCluster,2:3);
Mc = BuildAdjcancyMatrix(points, obstacles, params.connectionRadius);
PlotEnvironment(params, points, [], Mc, inspectionPoints, obstacles);
nPoints = size(points,1);
startIdx = randi(nPoints);
goalIdx = randi(nPoints);
% startPoint = points(startIdx, :);
% goalPoint = points(goalIdx, :);
% Run A*
pathFound = AStar(points, startIdx, goalIdx, Mc);
for k = 2:size(pathFound,1)
    plot([pathFound(k-1,2) pathFound(k,2)], [pathFound(k-1,3) pathFound(k,3)], '-g')
end