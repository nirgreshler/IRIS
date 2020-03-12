clear
close all
clc
set(0,'DefaultFigureWindowStyle','docked')

env_name = 'syn_9rooms';
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
params.maxClusters = 500;

params.laplacianType = 'normal';
clusters = SpectralClustering(params, points,M);
% clusters = InspectionClustering(params, points, pointsInSight, M);
clusterIdcs = unique(clusters);
nClusters = length(clusterIdcs);
% %% Reorganize the inspection Points
% newInspectionPoints = nan(size(inspectionPoints));
% pointsAlreadySeen = [];
% for clusterNum = 1:nClusters
%     pointsInCluster = clusters == clusterIdcs(clusterNum);
%     pointsSeenByCluster{clusterNum} = inspectionPoints(find(any(pointsInSight(pointsInCluster,:))));
%     nAlreadySeen = length(pointsAlreadySeen);
%     newPoints = setxor(pointsAlreadySeen, pointsSeenByCluster{clusterNum});
%     newPoints = intersect(newPoints, pointsSeenByCluster{clusterNum});
%     pointsAlreadySeen = [pointsAlreadySeen; newPoints];
%     nNewPoints = length(newPoints);
%     newInspectionPoints(nAlreadySeen+1:nAlreadySeen+nNewPoints) = newPoints;
% end
% newPointsInSight = zeros(size(pointsInSight));
% for k = 1:nPoints
%     idcs = find(~isnan(sights(k,:)));
%     inspectionThisPoint = sights(k,idcs);
%     [~, idcsOfInspectionPoints] = intersect(newInspectionPoints, inspectionThisPoint);
%     newPointsInSight(k, idcsOfInspectionPoints) = 1;
% end
% pointsInSight = newPointsInSight;

%% Check connectivity within clusters
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    Mc = M(pointsInCluster, pointsInCluster);
    D = diag(sum(Mc));
    L = D-Mc;
    assert(sum(abs(eig(L)) < 1e-6) == 1, 'Cluster is not connected (kashir)')
end
%% Examine inspection in each cluster
cInspectionPoints = cell(nClusters,1);
cInspectionPointsH = cell(nClusters,1);
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    [r,c] = find(pointsInSight(pointsInCluster,:));
    cInspectionPointsH{k} = c;
    cInspectionPoints{k} = sum(pointsInSight(pointsInCluster,:));
end
colorOrder = linspecer(nClusters);
%% Plot histogram of inspection
figure; hold all
set(gca, 'colorOrder', colorOrder)
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    coverage = 100*mean(cInspectionPoints{k} > 0);
    idcs = find(cInspectionPoints{k} > 0);
    plot(idcs, cInspectionPoints{k}(idcs), '.-', 'DisplayName', sprintf('Cluster #%d (%d points. Coverage:%.1f%%)', k, sum(pointsInCluster), coverage))
%     histogram(cInspectionPoints{k}, nInspectionPoints)
end
xlabel('Inoection Points')
title(['Coverage per Cluster (', num2str(nClusters), ' Clusters)'])
legend show
%% Plot histogram of inspection for each cluster
clusterScores = zeros(nClusters,1);
for k = 1:nClusters
    pointsInCluster = clusters == clusterIdcs(k);
    inspectionByThisCluster = unique(cInspectionPointsH{k});
    coverage = 100*mean(cInspectionPoints{k} > 0);
    nSeenByCluster = length(inspectionByThisCluster);
    nOccurs = zeros(nSeenByCluster,1);
    for p = 1:nSeenByCluster
        nOccurs(p) = sum(cInspectionPointsH{k} == inspectionByThisCluster(p));
    end
    % sort by occurrence
    nOccurs = sort(nOccurs);
    xForHist = [];
    for p = 1:nSeenByCluster
        xForHist = [xForHist; repmat(p, nOccurs(p), 1)];
    end 
    clusterScores(k) = (coverage/100)*nInspectionPoints/sum(pointsInCluster);
    if nSeenByCluster > 0
        figure; histogram(xForHist, nSeenByCluster);
        title(sprintf('Cluster %d. %d points. Coverage: %.1f%%\nCoverage score: %.2f',...
            k, sum(pointsInCluster), coverage, clusterScores(k)))
    end
end
%% Accumulating clusters by score
[~,i] = max(clusterScores);
accumIdcs = i;
coverageSoFar = any(cell2mat(cInspectionPoints(accumIdcs)),1);

[sortedScores, sIdcs] = sort(clusterScores, 'descend');
totalPoints = 0;
for k = 1:nClusters
    totalCoverage = mean(any(cell2mat(cInspectionPoints(sIdcs(1:k))) > 0, 1))*100;
    totalPoints = totalPoints+sum(clusters == clusterIdcs(sIdcs(k)));
    fprintf('After %d Clusters: %d points, %.1f%% Coverage.\n', k, totalPoints, totalCoverage)
end
%% Plot points
if contains(env_name, 'drone')
    figure; hold all
    set(gca, 'colorOrder', colorOrder)
    for k = 1:nClusters
        pointsInCluster = clusters == clusterIdcs(k);
        plot3(conf(pointsInCluster,2), conf(pointsInCluster,3), conf(pointsInCluster,4), '.', 'MarkerSize', 15, 'DisplayName', sprintf('Cluster #%d (%d points)', k, sum(pointsInCluster)))
    end
    xlabel('x'); ylabel('x'); zlabel('z');
    title('Configuration Space')
    legend show
    grid on
end