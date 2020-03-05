clear
close all
clc

env_name = 'planar_1000';
addpath(genpath(pwd))

base_name = fullfile(pwd, 'Graphs');

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));
nInspectionPoints = max(reshape(vertex(:,2:end),1,[]))+1;
M = Edges2M(edges(:,[1 2 7]));
points = conf(:,2:end);
nPoints = size(points,1);
pointsInSight = zeros(nPoints, nInspectionPoints);
sights = vertex(:,4:end);
for k = 1:nPoints
    idcs = find(~isnan(sights(k,:)));
    pointsInSight(k, sights(k,idcs)+1) = 1;
end
params.normalizeM = false;
% params.minClusters = 5;
% clusters = SpectralClustering(params, points,M);
clusters = InspectionClustering([], points, pointsInSight, M);

clusterIdcs = unique(clusters);
nClusters = length(clusterIdcs);
% Examine inspection in each cluster
cInspectionPoints = cell(nClusters,1);
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    cInspectionPoints{k} = sum(pointsInSight(pointsInCluster,:));
end
colorOrder = linspecer(nClusters);
figure; hold all
set(gca, 'colorOrder', colorOrder)
for k = 1:nClusters
    plot(1:nInspectionPoints, cInspectionPoints{k}, '.-')
end

