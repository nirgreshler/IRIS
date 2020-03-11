clear
close all
clc

env_name = 'drone_1000';
addpath(genpath(pwd))

base_name = fullfile(pwd, 'Graphs');

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));
inspectionPoints = (reshape(vertex(:,4:end),1,[]));
inspectionPoints = unique(inspectionPoints(~isnan(inspectionPoints)));
nInspectionPoints = length(inspectionPoints);
M = Edges2M(edges(:,[1 2 7]));
points = conf(:,2:end);
nPoints = size(points,1);
pointsInSight = zeros(nPoints, nInspectionPoints);
sights = vertex(:,4:end);
for k = 1:nPoints
    idcs = find(~isnan(sights(k,:)));
    inspectionThisPoint = sights(k,idcs);
    [~, idcsOfInspectionPoints] = intersect(inspectionPoints, inspectionThisPoint);
    pointsInSight(k, idcsOfInspectionPoints) = 1;
end
params.normalizeM = false;
params.minClusters = 1;
params.maxClusters = 100;

params.laplacianType = 'normal';
clusters = SpectralClustering(params, points,M);
% clusters = InspectionClustering([], points, pointsInSight, M);

clusterIdcs = unique(clusters);
nClusters = length(clusterIdcs);
%% Reorganize the inspection Points
newInspectionPoints = nan(size(inspectionPoints));
pointsAlreadySeen = [];
for clusterNum = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(clusterNum);
    pointsSeenByCluster{clusterNum} = inspectionPoints(find(any(pointsInSight(pointsInCluster,:))));
    nAlreadySeen = length(pointsAlreadySeen);
    newPoints = setxor(pointsAlreadySeen, pointsSeenByCluster{clusterNum});
    newPoints = intersect(newPoints, pointsSeenByCluster{clusterNum});
    pointsAlreadySeen = [pointsAlreadySeen; newPoints];
    nNewPoints = length(newPoints);
    newInspectionPoints(nAlreadySeen+1:nAlreadySeen+nNewPoints) = newPoints;
end
newPointsInSight = zeros(size(pointsInSight));
for k = 1:nPoints
    idcs = find(~isnan(sights(k,:)));
    inspectionThisPoint = sights(k,idcs);
    [~, idcsOfInspectionPoints] = intersect(newInspectionPoints, inspectionThisPoint);
    newPointsInSight(k, idcsOfInspectionPoints) = 1;
end

%% Examine inspection in each cluster
cInspectionPoints = cell(nClusters,1);
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
%     [r,c] = find(pointsInSight(pointsInCluster,:));
%     cInspectionPoints{k} = c;
    cInspectionPoints{k} = sum(newPointsInSight(pointsInCluster,:));
end
colorOrder = linspecer(nClusters);
%% Plot histogram of inspection
figure; hold all
set(gca, 'colorOrder', colorOrder)
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    plot(1:nInspectionPoints, cInspectionPoints{k}, '.-', 'DisplayName', sprintf('Cluster #%d (%d points)', k, sum(pointsInCluster)))
%     histogram(cInspectionPoints{k}, nInspectionPoints)
end
title('Coverage per Cluster')
legend show
title([num2str(nClusters), ' Clusters'])
%% Plot points
figure; hold all
set(gca, 'colorOrder', colorOrder)
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    plot3(conf(pointsInCluster,2), conf(pointsInCluster,3), conf(pointsInCluster,4), '.', 'DisplayName', sprintf('Cluster #%d (%d points)', k, sum(pointsInCluster)))
end
xlabel('x'); ylabel('x'); zlabel('z');
title('Configuration Space')
legend show
title([num2str(nClusters), ' Clusters'])
grid on