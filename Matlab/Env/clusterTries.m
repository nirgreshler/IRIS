clear
close all
clc

env_name = 'syn_9rooms';
addpath(genpath(pwd))

base_name = fullfile(pwd, 'Graphs');

[conf, vertex, edges] = read_graph(fullfile(base_name, env_name));
[obstacles, inspectionPoints, params] = read_graph_metadata(fullfile(base_name, env_name));
M = Edges2M(edges(:,[1 2 7]));
points = conf(:,2:3);
nPoints = size(points,1);
pointsInSight = GetPointsInSight(params, points, inspectionPoints, obstacles);
% Remove points with no inspection from clustering
invalidPointsIdcs = find(sum(pointsInSight,2) < size(inspectionPoints,1)*0.01);
validPointsIdcs = setxor(1:nPoints, invalidPointsIdcs);
nPoints = length(validPointsIdcs);
pointsInSight = pointsInSight(validPointsIdcs,:);

Ms = zeros(nPoints);
for k = 1:nPoints-1
    clc
    fprintf('Calculating jacard distance... %.1f%%\n', 100*(k/(nPoints-1)))
    for j = k+1:nPoints
        i1 = find(pointsInSight(k,:));
        i2 = find(pointsInSight(j,:));
        un = sum(pointsInSight(k,:) | pointsInSight(j,:));
        if un == 0
            Ms(k,j) = 0;
            Ms(j,k) = 0;
        else
            in = sum(pointsInSight(k,:) & pointsInSight(j,:));
            Ms(k,j) = in/un;
            Ms(j,k) = Ms(k,j);
        end
    end
end
D = diag(sum(Ms));
L = D-Ms;
[V,eigenMat] = eig(L);

eigenValues = diag(eigenMat);
eigenValues = eigenValues(abs(eigenValues) > 1e-6);
dEigens = diff(eigenValues);
[~, maxIdx] = max(dEigens);
nClusters = maxIdx;

kSmallestEigenvalues = eigenValues(1:nClusters+1);
kSmallestEigenvectors = V(:,1:nClusters+1);

z = zeros(nPoints, nClusters);
for ii = 1:nPoints
    z(ii,:) = 1./kSmallestEigenvalues(2:end).*kSmallestEigenvectors(ii,2:end)';
end
clusters = kmeans(z, nClusters);
% clusters = kmeans(kSmallestEigenvectors, nClusters);

% % Remove small clusters
% [N, edges] = histcounts(clusters, nClusters);
% smallClusters = find(N < 0.05*nPoints);
% largeClusters = setxor(1:nClusters,smallClusters);
% [~, pointsOfSmallClusters] = intersect(clusters, smallClusters);
% clusters(pointsOfSmallClusters) = nClusters+1;

clustersAll(validPointsIdcs) = clusters;
clustersAll(invalidPointsIdcs) = nClusters+1;
params.inspectInspection = true;

PlotEnvironment(params, points, clustersAll, M, inspectionPoints, obstacles, 'Clustered by Inspection');
