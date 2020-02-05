function [clustersKmeans, clustersSpectral] = ClusterPoints(points, M)
% kmean
D = diag(sum(M));
L = D-M;
[V,D] = eig(L);
eigenValues = diag(D);
dEigens = diff(eigenValues);
[~, maxIdx] = max(dEigens(1:round(length(dEigens)/2)));
nClusters = maxIdx;
clustersKmeans = kmeans(points, nClusters);
kSmallestEigenvectors = V(:,1:nClusters+1);
clustersSpectral = kmeans(kSmallestEigenvectors, nClusters);